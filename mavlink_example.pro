TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    #src/main.cpp \
    src/remotenode.cpp \
    src/main.cpp

HEADERS += \
    src/remotenode.h \
    src/mavlinkdefinitions.h
