QT       += core gui widgets

TARGET = OgreQT5
TEMPLATE = app

INCLUDEPATH *= /usr/include/OGRE

SOURCES += *.cpp
HEADERS  += *.hpp
FORMS    += MainWindow.ui

#DEFINES *= OGRE_NODELESS_POSITIONING

LIBS += -lboost_system
LIBS += -lOgreMain -lOgreBites

