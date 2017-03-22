#-------------------------------------------------
#
# Project created by QtCreator 2017-02-07T16:11:27
#
#-------------------------------------------------

QT       -= gui
QT += serialport
CONFIG += serialport

TARGET = gibicTrack
TEMPLATE = lib

DEFINES += GIBICTRACK_LIBRARY

SOURCES += gibictrack.cpp \
    ../../Pru-G1B1C 3FFT/RaabAlgorithm.cpp \
    ../../Pru-G1B1C 3FFT/RaabAlgorithm.cpp

HEADERS += gibictrack.h\
        gibicTrack_global.h \
    ../../Pru-G1B1C 3FFT/RaabAlgorithm.h

symbian {
    MMP_RULES += EXPORTUNFROZEN
    TARGET.UID3 = 0xE446955D
    TARGET.CAPABILITY = 
    TARGET.EPOCALLOWDLLDATA = 1
    addFiles.sources = gibicTrack.dll
    addFiles.path = !:/sys/bin
    DEPLOYMENT += addFiles
}

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
