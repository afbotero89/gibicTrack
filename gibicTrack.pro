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
    RaabAlgorithm.cpp \
    data_man.cpp

HEADERS += gibictrack.h\
        gibicTrack_global.h \
        RaabAlgorithm.h \
    data_man.h

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



win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../Developer/OpenIGTLink-build/bin/release/ -lOpenIGTLink
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../Developer/OpenIGTLink-build/bin/debug/ -lOpenIGTLink
else:symbian: LIBS += -lOpenIGTLink
else:unix: LIBS += -L$$PWD/../../Developer/OpenIGTLink-build/bin/ -lOpenIGTLink

INCLUDEPATH += $$PWD/../../Developer/OpenIGTLink-build/bin
DEPENDPATH += $$PWD/../../Developer/OpenIGTLink-build/bin

win32:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../Developer/OpenIGTLink-build/bin/release/OpenIGTLink.lib
else:win32:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../Developer/OpenIGTLink-build/bin/debug/OpenIGTLink.lib
else:unix:!symbian: PRE_TARGETDEPS += $$PWD/../../Developer/OpenIGTLink-build/bin/libOpenIGTLink.a
