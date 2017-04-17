TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
LIBS += \
    -lprotobuf \
    -lkrpc

SOURCES += \
    pid.cpp \
    quad_vesselcontrol.cpp \
    tuple_operations.cpp \
    src/main_skycrane.cpp \
    src/pid.cpp \
    src/quad_vesselcontrol.cpp \
    src/tuple_operations.cpp

HEADERS += \
    pid.h \
    quad_vesselcontrol.h \
    tuple_operations.h \
    include/pid.h \
    include/quad_vesselcontrol.h \
    include/tuple_operations.h
