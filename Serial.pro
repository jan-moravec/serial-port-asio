TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        library/serial.cpp \
        main.cpp

HEADERS += \
    library/serial.h

win32 {
    INCLUDEPATH += C:/Boost/include/boost-1_70
    LIBS += -L C:/Boost/lib -lboost_thread-mgw73-mt-x64-1_70 -lboost_system-mgw73-mt-x64-1_70  \
            -lboost_timer-mgw73-mt-x64-1_70 -lboost_exception-mgw73-mt-x64-1_70 -lboost_filesystem-mgw73-mt-x64-1_70 \
            -lboost_serialization-mgw73-mt-x64-1_70 -lboost_coroutine-mgw73-mt-x64-1_70 -lboost_atomic-mgw73-mt-x64-1_70 \
            -lboost_chrono-mgw73-mt-x64-1_70 -lboost_wserialization-mgw73-mt-x64-1_70 -lws2_32 -lwsock32
}

unix {
    LIBS += -lboost_thread -lboost_system -lboost_timer -lboost_serialization -lboost_coroutine -lboost_atomic \
            -lboost_chrono -lboost_wserialization -lpthread
}
