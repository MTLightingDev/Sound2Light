cmake_minimum_required(VERSION 3.16)

project(Sound2Light VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable automatic handling of Qt MOC, UIC, and RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set (CMAKE_PREFIX_PATH "/opt/homebrew/opt/qt@5")

# Find Qt5 packages
find_package(Qt5 REQUIRED COMPONENTS
    Core
    Qml
    Quick
    Multimedia
    QuickWidgets
    Network
    Widgets
)

# Source files
set(SOURCES
    src/main.cpp
    src/FFTAnalyzer.cpp
    src/MainController.cpp
    src/MonoAudioBuffer.cpp
    src/QAudioInputWrapper.cpp
    src/ScaledSpectrum.cpp
    src/TriggerFilter.cpp
    src/OSCParser.cpp
    src/TriggerGenerator.cpp
    src/TriggerGuiController.cpp
    src/TriggerOscParameters.cpp
    src/OSCMessage.cpp
    src/OSCMapping.cpp
    src/OSCNetworkManager.cpp
    src/BPMDetector.cpp
    src/BPMOscControler.cpp
    src/BPMTapDetector.cpp
)

# Header files
set(HEADERS
    src/ffft/Array.h
    src/ffft/Array.hpp
    src/ffft/def.h
    src/ffft/DynArray.h
    src/ffft/DynArray.hpp
    src/ffft/FFTReal.h
    src/ffft/FFTReal.hpp
    src/ffft/FFTRealFixLen.h
    src/ffft/FFTRealFixLen.hpp
    src/ffft/FFTRealFixLenParam.h
    src/ffft/FFTRealPassDirect.h
    src/ffft/FFTRealPassDirect.hpp
    src/ffft/FFTRealPassInverse.h
    src/ffft/FFTRealPassInverse.hpp
    src/ffft/FFTRealSelect.h
    src/ffft/FFTRealSelect.hpp
    src/ffft/FFTRealUseTrigo.h
    src/ffft/FFTRealUseTrigo.hpp
    src/ffft/OscSinCos.h
    src/ffft/OscSinCos.hpp
    src/AudioInputInterface.h
    src/BasicFFTInterface.h
    src/FFTAnalyzer.h
    src/FFTRealWrapper.h
    src/MainController.h
    src/MonoAudioBuffer.h
    src/QAudioInputWrapper.h
    src/ScaledSpectrum.h
    src/TriggerGeneratorInterface.h
    src/TriggerFilter.h
    src/OSCParser.h
    src/utils.h
    src/TriggerGenerator.h
    src/TriggerGuiController.h
    src/TriggerOscParameters.h
    src/versionInfo.h
    src/OSCMessage.h
    src/OSCMapping.h
    src/OSCNetworkManager.h
    src/CircularBuffer.h
    src/BPMDetector.h
    src/BPMOscControler.h
    src/BPMTapDetector.h
)

# Resource files
set(RESOURCES
    src/qml.qrc
    src/images.qrc
)

# Create the executable
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${RESOURCES}
)

# Link Qt5 libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt5::Core
    Qt5::Qml
    Qt5::Quick
    Qt5::Multimedia
    Qt5::QuickWidgets
    Qt5::Network
    Qt5::Widgets
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ffft
)

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall)
endif()

