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

    // create new input:
    m_activeInputName = selectedDevice.deviceName();
    m_audioInput = new QAudioInput(selectedDevice, m_actualAudioFormat, this);
    m_audioInput->setVolume(1.0);
    m_audioIODevice = m_audioInput->start();
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
	// read data from input as QByteArray:
    QByteArray data = m_audioIODevice->readAll();

    const int bytesPerSample = m_actualAudioFormat.sampleSize() / 8; // Qt5 API
    if (bytesPerSample <= 0) return;

    const std::size_t numSamples = std::size_t(data.size()) / std::size_t(bytesPerSample);
    QVector<qreal> realData(numSamples);

    const char *ptr = data.constData();
    const bool isLittleEndian = (m_actualAudioFormat.byteOrder() == QAudioFormat::LittleEndian);

    for (std::size_t i = 0; i < numSamples; ++i) {
        qreal scaled = 0.0;
        switch (m_actualAudioFormat.sampleType()) {
        case QAudioFormat::SignedInt:
            if (bytesPerSample == 2) {
                qint16 pcm = isLittleEndian
                    ? qFromLittleEndian<qint16>(reinterpret_cast<const uchar*>(ptr))
                    : qFromBigEndian<qint16>(reinterpret_cast<const uchar*>(ptr));
                scaled = qreal(pcm) / 32768.0;
            } else if (bytesPerSample == 4) {
                qint32 pcm = isLittleEndian
                    ? qFromLittleEndian<qint32>(reinterpret_cast<const uchar*>(ptr))
                    : qFromBigEndian<qint32>(reinterpret_cast<const uchar*>(ptr));
                scaled = qreal(pcm) / 2147483648.0; // 2^31
            } else if (bytesPerSample == 1) {
                // 8-bit signed PCM (rare)
                const qint8 pcm = *reinterpret_cast<const qint8*>(ptr);
                scaled = qreal(pcm) / 128.0;
            }
            break;
        case QAudioFormat::UnSignedInt:
            if (bytesPerSample == 1) {
                const quint8 pcm = *reinterpret_cast<const quint8*>(ptr);
                scaled = (qreal(int(pcm) - 128) / 128.0);
            } else if (bytesPerSample == 2) {
                quint16 pcm = isLittleEndian
                    ? qFromLittleEndian<quint16>(reinterpret_cast<const uchar*>(ptr))
                    : qFromBigEndian<quint16>(reinterpret_cast<const uchar*>(ptr));
                // center at 0 and scale
                scaled = (qreal(int(pcm) - 32768) / 32768.0);
            } else if (bytesPerSample == 4) {
                quint32 pcm = isLittleEndian
                    ? qFromLittleEndian<quint32>(reinterpret_cast<const uchar*>(ptr))
                    : qFromBigEndian<quint32>(reinterpret_cast<const uchar*>(ptr));
                scaled = (qreal(qint64(pcm) - 2147483648LL) / 2147483648.0);
            }
            break;
        case QAudioFormat::Float:
            if (bytesPerSample == 4) {
                quint32 raw = isLittleEndian
                    ? qFromLittleEndian<quint32>(reinterpret_cast<const uchar*>(ptr))
                    : qFromBigEndian<quint32>(reinterpret_cast<const uchar*>(ptr));
                float f;
                std::memcpy(&f, &raw, sizeof(float));
                scaled = qreal(f);
            }
            break;
        default:
            scaled = 0.0;
            break;
        }
        realData[i] = scaled;
        ptr += bytesPerSample;
    }

    // Call MonoAudioBuffer as next element in the processing chain:
	m_buffer->putSamples(realData, m_actualAudioFormat.channelCount());
}
