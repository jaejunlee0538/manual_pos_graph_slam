#-------------------------------------------------
#
# Project created by QtCreator 2016-10-11T21:34:50
#
#-------------------------------------------------

QT       += core gui xml opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += qt opengl thread console no_keywords c++11

TARGET = abc

TEMPLATE = app

G2O_INSTALL_PATH = $$PWD/Externals/g2o/install
EIGEN_INSTALL_PATH = /usr/include/eigen3


INCLUDEPATH += "/usr/local/include"
INCLUDEPATH += "/usr/include/pcl-1.7"
INCLUDEPATH += "/usr/include/suitesparse"
INCLUDEPATH += $$G2O_INSTALL_PATH/include
INCLUDEPATH += $$EIGEN_INSTALL_PATH

SOURCES += main.cpp\
        mainwindow.cpp \
    cloudviewer.cpp \
    pointclouddisplayer.cpp \
    graphdisplayer.cpp \
    graphslam.cpp \
    Global.cpp \
    manipulatedframesetconstraint.cpp \
    graphhelper.cpp \
    standardcamera.cpp

HEADERS  += mainwindow.h \
    cloudviewer.h\
    pointclouddisplayer.h \
    graphdisplayer.h \
    graphslam.h \
    postypes.h \
    exceptions.h \
    Logger.h \
    LoggerQEditBox.h \
    Global.h \
    manipulatedframesetconstraint.h \
    graphhelper.h \
    standardcamera.h

unix {
#        CONFIG -= debug debug_and_release
#        CONFIG *= release
        equals (QT_MAJOR_VERSION, 4) {
            LIB_NAME = QGLViewer-qt4
        }
        equals (QT_MAJOR_VERSION, 5) {
            LIB_NAME = QGLViewer-qt5
        }

        LIBS += -L"/usr/local/lib" -l$${LIB_NAME}

        # Intermediate files are created in hidden folders
        MOC_DIR = .moc
        OBJECTS_DIR = .obj
}


LIBS += -L$$G2O_INSTALL_PATH/lib
LIBS += -lg2o_types_data -lg2o_types_slam2d -lg2o_types_slam3d -lg2o_types_slam3d_addons -lg2o_solver_pcg -lg2o_solver_dense -lg2o_solver_cholmod -lg2o_solver_csparse -lg2o_csparse_extension -lg2o_core -lg2o_stuff -lg2o_opengl_helper -lg2o_ext_freeglut_minimal
LIBS += -lGLU
LIBS += -lcxsparse
LIBS += -lpcl_common -lpcl_io
LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

FORMS    += mainwindow.ui
