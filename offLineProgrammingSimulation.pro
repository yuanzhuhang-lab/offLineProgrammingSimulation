QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG -= debug_and_release

QMAKE_CXXFLAGS += /MP

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

QMAKE_CXXFLAGS_RELEASE = -ZI -MD
QMAKE_LFLAGS_RELEASE = /DEBUG

include(./3rdparty/3rdparty.pri)
include( ./src/src.pri )


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

INCLUDEPATH += $$PWD/./src
DEPENDPATH += $$PWD/./src
message("====================$$INCLUDEPATH")

# 指定构建目录
DESTDIR = $$OUT_PWD/build
# 指定对象文件目录
OBJECTS_DIR = $$DESTDIR/obj
# 指定moc文件目录
MOC_DIR = $$DESTDIR/moc
# 指定uic文件目录
UI_DIR = $$DESTDIR/ui
# 指定rcc文件目录
RCC_DIR = $$DESTDIR/qrc
# 指定程序预编译数据库文件路径
QMAKE_LFLAGS += /PDB:$$DESTDIR/$${TARGET}.pdb
QMAKE_CXXFLAGS += /Fd"$$DESTDIR/"

QMAKE_CXXFLAGS += /bigobj
