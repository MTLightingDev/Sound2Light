// Copyright (c) 2016 Electronic Theatre Controls, Inc., http://www.etcconnect.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "QAudioInputWrapper.h"

#include "utils.h"

#include <QList>
#include <QVector>
#include <QByteArray>
#include <QDebug>
#include <QtMultimedia/QAudioFormat>
#include <QtMultimedia/QAudioInput>
#include <QtMultimedia/QAudioDeviceInfo>

#include <QtEndian>
#include <QElapsedTimer>
#include <algorithm>
#include <cmath>
#include <cstring>

QAudioInputWrapper::QAudioInputWrapper(MonoAudioBuffer *buffer)
    : AudioInputInterface(buffer)
    , m_audioInput(nullptr)
    , m_audioIODevice(nullptr)
{
    // Set up the desired format for mono audio input (Qt5 API)
    m_desiredAudioFormat.setSampleRate(44100);
    m_desiredAudioFormat.setChannelCount(1);  // mono
    m_desiredAudioFormat.setSampleSize(16);
    m_desiredAudioFormat.setSampleType(QAudioFormat::SignedInt);
    m_desiredAudioFormat.setByteOrder(QAudioFormat::LittleEndian);
}

QAudioInputWrapper::~QAudioInputWrapper()
{
	// Close and delete previous input device:
	if (m_audioInput) {
		m_audioInput->stop();
	}
	if (m_audioIODevice && m_audioIODevice->isOpen()) {
		m_audioIODevice->close();
	}
	delete m_audioInput;
	m_audioInput = nullptr;
	m_audioIODevice = nullptr;
}

QStringList QAudioInputWrapper::getAvailableInputs() const
{
    // get List of input device names from Qt5 API (QAudioDeviceInfo)
    QStringList deviceList;
    const QList<QAudioDeviceInfo> devices = QAudioDeviceInfo::availableDevices(QAudio::AudioInput);
    for (const QAudioDeviceInfo &device : devices) {
        deviceList.append(device.deviceName());
    }
    return deviceList;
}

QString QAudioInputWrapper::getDefaultInputName() const
{
    const QAudioDeviceInfo defaultDevice = QAudioDeviceInfo::defaultInputDevice();
    return defaultDevice.isNull() ? QString("") : defaultDevice.deviceName();
}

void QAudioInputWrapper::setInputByName(const QString &inputName)
{
    // Close and delete previous input device:
    if (m_audioInput) {
        m_audioInput->stop();
    }
    if (m_audioIODevice && m_audioIODevice->isOpen()) {
        disconnect(m_audioIODevice, &QIODevice::readyRead, this, &QAudioInputWrapper::audioDataReady);
        m_audioIODevice->close();
    }
    delete m_audioInput;
    m_audioInput = 0;

    // Get device info of new input (Qt5 API):
    const QList<QAudioDeviceInfo> devices = QAudioDeviceInfo::availableDevices(QAudio::AudioInput);
    QAudioDeviceInfo selectedDevice = QAudioDeviceInfo::defaultInputDevice();
    for (const QAudioDeviceInfo &device : devices) {
        if (device.deviceName() == inputName) {
            selectedDevice = device;
            break;
        }
    }

    // check if desired format is supported:
    if (!selectedDevice.isFormatSupported(m_desiredAudioFormat)) {
        qWarning() << "Desired audio format not supported, using preferred format.";
        qWarning() << "Current device:" << selectedDevice.preferredFormat();
        m_actualAudioFormat = selectedDevice.preferredFormat();
    } else {
        m_actualAudioFormat = m_desiredAudioFormat;
    }

    // Precompute decode params for branchless inner loop
    m_bytesPerSample = m_actualAudioFormat.sampleSize() / 8;
    m_channelCount   = m_actualAudioFormat.channelCount();
    m_isLittleEndian = (m_actualAudioFormat.byteOrder() == QAudioFormat::LittleEndian);
    m_decodeKind = DecodeKind::Unsupported;
    switch (m_actualAudioFormat.sampleType()) {
    case QAudioFormat::SignedInt:
        if (m_bytesPerSample == 1) m_decodeKind = DecodeKind::S8;
        else if (m_bytesPerSample == 2) m_decodeKind = DecodeKind::S16;
        else if (m_bytesPerSample == 3) m_decodeKind = DecodeKind::S24;
        else if (m_bytesPerSample == 4) m_decodeKind = DecodeKind::S32;
        break;
    case QAudioFormat::UnSignedInt:
        if (m_bytesPerSample == 1) m_decodeKind = DecodeKind::U8;
        else if (m_bytesPerSample == 2) m_decodeKind = DecodeKind::U16;
        else if (m_bytesPerSample == 3) m_decodeKind = DecodeKind::U24;
        else if (m_bytesPerSample == 4) m_decodeKind = DecodeKind::U32;
        break;
    case QAudioFormat::Float:
        if (m_bytesPerSample == 4) m_decodeKind = DecodeKind::F32;
        break;
    default:
        m_decodeKind = DecodeKind::Unsupported;
        qWarning() << "Unsupported sample type:" << m_actualAudioFormat.sampleType();
        break;
    }

    // Log selected format and decode path for verification
    auto stToStr = [](QAudioFormat::SampleType st){
        switch (st) {
            case QAudioFormat::SignedInt: return "SignedInt";
            case QAudioFormat::UnSignedInt: return "UnSignedInt";
            case QAudioFormat::Float: return "Float";
            default: return "Unknown";
        }
    };
    auto boToStr = [](QAudioFormat::Endian bo){ return bo == QAudioFormat::LittleEndian ? "LE" : "BE"; };
    auto dkToStr = [](DecodeKind dk){
        switch (dk) {
            case DecodeKind::S8: return "S8"; case DecodeKind::S16: return "S16"; case DecodeKind::S24: return "S24"; case DecodeKind::S32: return "S32";
            case DecodeKind::U8: return "U8"; case DecodeKind::U16: return "U16"; case DecodeKind::U24: return "U24"; case DecodeKind::U32: return "U32";
            case DecodeKind::F32: return "F32"; default: return "Unsupported";
        }
    };
    qInfo().nospace() << "Audio input format: rate=" << m_actualAudioFormat.sampleRate()
                      << " Hz, ch=" << m_actualAudioFormat.channelCount()
                      << ", size=" << m_actualAudioFormat.sampleSize() << "bits"
                      << ", type=" << stToStr(m_actualAudioFormat.sampleType())
                      << ", order=" << boToStr(m_actualAudioFormat.byteOrder())
                      << ", decode=" << dkToStr(m_decodeKind);

    // create new input:
    m_activeInputName = selectedDevice.deviceName();
    m_audioInput = new QAudioInput(selectedDevice, m_actualAudioFormat, this);

    // Basic diagnostics: log state changes and attempt gentle recovery from Idle/Suspended
    connect(m_audioInput, &QAudioInput::stateChanged, this, [this](QAudio::State s){
        const QAudio::Error err = m_audioInput ? m_audioInput->error() : QAudio::NoError;
        qInfo() << "QAudioInput state:" << s << ", error:" << err;
        if (!m_audioInput) return;
        switch (s) {
        case QAudio::IdleState:
            if (err == QAudio::NoError) {
                // Some backends flip to Idle when buffer underruns; try resuming once
                qInfo() << "IdleState with NoError; attempting resume()";
                m_audioInput->resume();
            } else {
                qWarning() << "IdleState due to error" << err;
            }
            break;
        case QAudio::SuspendedState:
            if (err == QAudio::NoError) {
                qInfo() << "SuspendedState with NoError; attempting resume()";
                m_audioInput->resume();
            }
            break;
        default:
            break;
        }
    });

    // Notify callback ~30ms to verify cadence
    m_audioInput->setNotifyInterval(30);
    connect(m_audioInput, &QAudioInput::notify, this, [](){ qInfo() << "QAudioInput notify()"; });

    // Set buffer size to reduce wakeups (tune as needed): ~1024 frames
    const int bytesPerFrame = qMax(1, m_bytesPerSample) * qMax(1, m_channelCount);
    m_audioInput->setBufferSize(1024 * bytesPerFrame);

    m_audioInput->setVolume(1.0);
    m_audioIODevice = m_audioInput->start();
    if (!m_audioIODevice) {
        qWarning() << "QAudioInput start() returned null I/O device. Error:" << m_audioInput->error();
        return;
    }

    // Reserve IO buffer to typical size (will grow if needed)
    m_ioBuf.reserve(m_audioInput->bufferSize());

    connect(m_audioIODevice, &QIODevice::readyRead, this, &QAudioInputWrapper::audioDataReady);
}

qreal QAudioInputWrapper::getVolume() const
{
	if (!m_audioInput) return 0.0;
	return m_audioInput->volume();
}

void QAudioInputWrapper::setVolume(const qreal &value)
{
	if (!m_audioInput) return;
	m_audioInput->setVolume(limit(0, value, 1));
}

void QAudioInputWrapper::audioDataReady()
{
    if (!m_audioIODevice || m_bytesPerSample <= 0) return;

    // Process all available complete frames to avoid backend Idle due to backlog
    const std::size_t bytesPerFrame = static_cast<std::size_t>(m_bytesPerSample) * static_cast<std::size_t>(qMax(1, m_channelCount));
    if (bytesPerFrame == 0) return;

    // Stats accumulators (windowed ~0.5s)
    static QElapsedTimer sTimer;
    static double accumSumSq = 0.0;
    static double accumPeak = 0.0;
    static quint64 accumSamples = 0;
    static quint64 accumBytes = 0;
    if (!sTimer.isValid()) sTimer.start();

    while (true) {
        const qint64 avail = m_audioIODevice->bytesAvailable();
        if (avail < static_cast<qint64>(bytesPerFrame)) break; // wait for full frame group

        // Read only full frames
        const qint64 framesToRead = (avail / static_cast<qint64>(bytesPerFrame));
        const qint64 toReadBytes = framesToRead * static_cast<qint64>(bytesPerFrame);
        if (toReadBytes <= 0) break;

        // Ensure IO buffer capacity and read
        if (m_ioBuf.size() < toReadBytes) m_ioBuf.resize(static_cast<int>(toReadBytes));
        const qint64 nread = m_audioIODevice->read(m_ioBuf.data(), static_cast<int>(toReadBytes));
        if (nread <= 0) break; // nothing read

        const std::size_t totalFrames = static_cast<std::size_t>(nread) / bytesPerFrame;
        const std::size_t numSamples = totalFrames * static_cast<std::size_t>(qMax(1, m_channelCount));
        if (numSamples == 0) continue;

        // Ensure reusable real buffer has enough capacity and size
        if (m_realBuf.size() < static_cast<int>(numSamples)) m_realBuf.resize(static_cast<int>(numSamples));

        const uchar* ptr = reinterpret_cast<const uchar*>(m_ioBuf.constData());

        switch (m_decodeKind) {
        case DecodeKind::S8:
            for (std::size_t i = 0; i < numSamples; ++i) {
                const qint8 v = *reinterpret_cast<const qint8*>(ptr);
                m_realBuf[static_cast<int>(i)] = qreal(v) / 128.0;
                ptr += 1;
            }
            break;
        case DecodeKind::S16:
            if (m_isLittleEndian) {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    const qint16 v = qFromLittleEndian<qint16>(ptr);
                    m_realBuf[static_cast<int>(i)] = qreal(v) / 32768.0;
                    ptr += 2;
                }
            } else {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    const qint16 v = qFromBigEndian<qint16>(ptr);
                    m_realBuf[static_cast<int>(i)] = qreal(v) / 32768.0;
                    ptr += 2;
                }
            }
            break;
        case DecodeKind::S32:
            if (m_isLittleEndian) {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    const qint32 v = qFromLittleEndian<qint32>(ptr);
                    m_realBuf[static_cast<int>(i)] = qreal(v) / 2147483648.0; // 2^31
                    ptr += 4;
                }
            } else {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    const qint32 v = qFromBigEndian<qint32>(ptr);
                    m_realBuf[static_cast<int>(i)] = qreal(v) / 2147483648.0; // 2^31
                    ptr += 4;
                }
            }
            break;
        case DecodeKind::S24:
            if (m_isLittleEndian) {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    qint32 v = (qint32(ptr[0]) & 0xFF) | ((qint32(ptr[1]) & 0xFF) << 8) | ((qint32(ptr[2]) & 0xFF) << 16);
                    if (v & 0x00800000) v |= 0xFF000000; // sign-extend 24-bit
                    m_realBuf[static_cast<int>(i)] = qreal(v) / 8388608.0; // 2^23
                    ptr += 3;
                }
            } else {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    qint32 v = ((qint32(ptr[0]) & 0xFF) << 16) | ((qint32(ptr[1]) & 0xFF) << 8) | (qint32(ptr[2]) & 0xFF);
                    if (v & 0x00800000) v |= 0xFF000000;
                    m_realBuf[static_cast<int>(i)] = qreal(v) / 8388608.0;
                    ptr += 3;
                }
            }
            break;
        case DecodeKind::U8:
            for (std::size_t i = 0; i < numSamples; ++i) {
                const quint8 v = *ptr++;
                m_realBuf[static_cast<int>(i)] = qreal(int(v) - 128) / 128.0;
            }
            break;
        case DecodeKind::U16:
            if (m_isLittleEndian) {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    const quint16 v = qFromLittleEndian<quint16>(ptr);
                    m_realBuf[static_cast<int>(i)] = qreal(int(v) - 32768) / 32768.0;
                    ptr += 2;
                }
            } else {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    const quint16 v = qFromBigEndian<quint16>(ptr);
                    m_realBuf[static_cast<int>(i)] = qreal(int(v) - 32768) / 32768.0;
                    ptr += 2;
                }
            }
            break;
        case DecodeKind::U32:
            if (m_isLittleEndian) {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    const quint32 v = qFromLittleEndian<quint32>(ptr);
                    m_realBuf[static_cast<int>(i)] = qreal(qint64(v) - 2147483648LL) / 2147483648.0;
                    ptr += 4;
                }
            } else {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    const quint32 v = qFromBigEndian<quint32>(ptr);
                    m_realBuf[static_cast<int>(i)] = qreal(qint64(v) - 2147483648LL) / 2147483648.0;
                    ptr += 4;
                }
            }
            break;
        case DecodeKind::U24:
            if (m_isLittleEndian) {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    quint32 v = (quint32(ptr[0]) & 0xFF) | ((quint32(ptr[1]) & 0xFF) << 8) | ((quint32(ptr[2]) & 0xFF) << 16);
                    qint32 centered = qint32(v) - 0x00800000; // center at 0 (2^23 mid)
                    m_realBuf[static_cast<int>(i)] = qreal(centered) / 8388608.0; // 2^23
                    ptr += 3;
                }
            } else {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    quint32 v = ((quint32(ptr[0]) & 0xFF) << 16) | ((quint32(ptr[1]) & 0xFF) << 8) | (quint32(ptr[2]) & 0xFF);
                    qint32 centered = qint32(v) - 0x00800000;
                    m_realBuf[static_cast<int>(i)] = qreal(centered) / 8388608.0;
                    ptr += 3;
                }
            }
            break;
        case DecodeKind::F32:
            if (m_isLittleEndian) {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    const quint32 raw = qFromLittleEndian<quint32>(ptr);
                    float f;
                    std::memcpy(&f, &raw, sizeof(float));
                    m_realBuf[static_cast<int>(i)] = qreal(f);
                    ptr += 4;
                }
            } else {
                for (std::size_t i = 0; i < numSamples; ++i) {
                    const quint32 raw = qFromBigEndian<quint32>(ptr);
                    float f;
                    std::memcpy(&f, &raw, sizeof(float));
                    m_realBuf[static_cast<int>(i)] = qreal(f);
                    ptr += 4;
                }
            }
            break;
        case DecodeKind::Unsupported:
        default: {
            const bool isLE = m_isLittleEndian;
            const int bps = m_bytesPerSample;
            QAudioFormat::SampleType st = m_actualAudioFormat.sampleType();
            const uchar* p = reinterpret_cast<const uchar*>(m_ioBuf.constData());
            for (std::size_t i = 0; i < numSamples; ++i) {
                qreal scaled = 0.0;
                switch (st) {
                case QAudioFormat::SignedInt:
                    if (bps == 1) {
                        const qint8 v = *reinterpret_cast<const qint8*>(p);
                        scaled = qreal(v) / 128.0;
                    } else if (bps == 2) {
                        const qint16 v = isLE ? qFromLittleEndian<qint16>(p) : qFromBigEndian<qint16>(p);
                        scaled = qreal(v) / 32768.0;
                    } else if (bps == 3) {
                        qint32 v = isLE
                            ? (qint32(p[0]) & 0xFF) | ((qint32(p[1]) & 0xFF) << 8) | ((qint32(p[2]) & 0xFF) << 16)
                            : ((qint32(p[0]) & 0xFF) << 16) | ((qint32(p[1]) & 0xFF) << 8) | (qint32(p[2]) & 0xFF);
                        if (v & 0x00800000) v |= 0xFF000000; // sign-extend
                        scaled = qreal(v) / 8388608.0; // 2^23
                    } else if (bps == 4) {
                        const qint32 v = isLE ? qFromLittleEndian<qint32>(p) : qFromBigEndian<qint32>(p);
                        scaled = qreal(v) / 2147483648.0; // 2^31
                    }
                    break;
                case QAudioFormat::UnSignedInt:
                    if (bps == 1) {
                        const quint8 v = *p;
                        scaled = qreal(int(v) - 128) / 128.0;
                    } else if (bps == 2) {
                        const quint16 v = isLE ? qFromLittleEndian<quint16>(p) : qFromBigEndian<quint16>(p);
                        scaled = qreal(int(v) - 32768) / 32768.0;
                    } else if (bps == 3) {
                        quint32 v = isLE
                            ? (quint32(p[0]) & 0xFF) | ((quint32(p[1]) & 0xFF) << 8) | ((quint32(p[2]) & 0xFF) << 16)
                            : ((quint32(p[0]) & 0xFF) << 16) | ((quint32(p[1]) & 0xFF) << 8) | (quint32(p[2]) & 0xFF);
                        qint32 centered = qint32(v) - 0x00800000;
                        scaled = qreal(centered) / 8388608.0;
                    } else if (bps == 4) {
                        const quint32 v = isLE ? qFromLittleEndian<quint32>(p) : qFromBigEndian<quint32>(p);
                        scaled = qreal(qint64(v) - 2147483648LL) / 2147483648.0;
                    }
                    break;
                case QAudioFormat::Float:
                    if (bps == 4) {
                        const quint32 raw = isLE ? qFromLittleEndian<quint32>(p) : qFromBigEndian<quint32>(p);
                        float f;
                        std::memcpy(&f, &raw, sizeof(float));
                        scaled = qreal(f);
                    }
                    break;
                default:
                    scaled = 0.0;
                    break;
                }
                m_realBuf[static_cast<int>(i)] = scaled;
                p += bps;
            }
            break;
        }
        }

        // accumulate stats for this chunk
        double localPeak = 0.0;
        double localSumSq = 0.0;
        const int N = static_cast<int>(numSamples);
        for (int i = 0; i < N; ++i) {
            const double v = static_cast<double>(m_realBuf[i]);
            localPeak = std::max(localPeak, std::abs(v));
            localSumSq += v * v;
        }
        accumPeak = std::max(accumPeak, localPeak);
        accumSumSq += localSumSq;
        accumSamples += static_cast<quint64>(N);
        accumBytes += static_cast<quint64>(nread);

        // Pass to MonoAudioBuffer (it will downmix to mono if needed)
        m_buffer->putSamples(m_realBuf, m_channelCount);
    }

    if (sTimer.elapsed() >= 500) { // every ~0.5s
        const double rms = accumSamples ? std::sqrt(accumSumSq / static_cast<double>(accumSamples)) : 0.0;
        qInfo().nospace() << "Audio stats: blocks~0.5s, bytes=" << accumBytes
                          << ", samples=" << accumSamples
                          << ", ch=" << m_channelCount
                          << ", peak=" << QString::number(accumPeak, 'f', 3)
                          << ", rms=" << QString::number(rms, 'f', 3);
        // reset window
        accumSumSq = 0.0;
        accumPeak = 0.0;
        accumSamples = 0;
        accumBytes = 0;
        sTimer.restart();
    }
}
