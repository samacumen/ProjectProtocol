TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    #src/main.cpp \
    src/remotenode.cpp \
    src/main.cpp \
    src/mavlinknode.cpp

HEADERS += \
    src/remotenode.h \
    src/mavlinkdefinitions.h \
    src/mavlinknode.h

FORMS += \
    src/mavlinknode.ui
