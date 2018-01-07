#-------------------------------------------------
#
# Project created by QtCreator 2013-11-16T19:42:26
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = PED4
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app
INCLUDEPATH += /usr/local/Aria/include
QMAKE_LIBDIR = /usr/local/Aria/lib
LIBS += -lAria

SOURCES += main.cpp \
    control.cpp

HEADERS += \
    control.h
