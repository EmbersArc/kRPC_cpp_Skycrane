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
    main_skycrane.cpp

HEADERS += \
    pid.h \
    quad_vesselcontrol.h \
    tuple_operations.h \
