QT       += core gui network serialport printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    color_utils.cpp \
    main.cpp \
    mainwindow.cpp \
    matoperator.cpp \
    realsensecapture.cpp \
    uart.cpp \
    udp.cpp \
    yrc1000micro_com.cpp \
    yrc1000micro_command.cpp

HEADERS += \
    color_utils.h \
    mainwindow.h \
    matoperator.h \
    realsensecapture.h \
    uart.h \
    udp.h \
    yrc1000micro_com.h \
    yrc1000micro_command.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../3rd-party/opencv/lib/ -lopencv_world412
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../3rd-party/opencv/lib/ -lopencv_world412d

INCLUDEPATH += $$PWD/../3rd-party/opencv/include
DEPENDPATH += $$PWD/../3rd-party/opencv/include

win32: LIBS += -L$$PWD/../3rd-party/librealsense/lib/ -lrealsense2

INCLUDEPATH += $$PWD/../3rd-party/librealsense/include
DEPENDPATH += $$PWD/../3rd-party/librealsense/include
