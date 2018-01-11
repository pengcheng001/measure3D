TEMPLATE = app
CONFIG += C++11
QMAKE_CXXFLAGS += -std=gnu++11
CONFIG -= app_bundle
CONFIG -= qt

QT -= core gui
#greaterThan(QT_MAJOR_VERSION,4):QT += widgets

DESTDIR += ../bin
OBJECTS_DIR += ../objects

SOURCES += \
    ../../neolixai/common/MatViewer.cpp \
    ../../neolixai/config/config.cpp \
    ../../neolixai/config/hardDebug.cpp \
    ../../neolixai/imagepro/CalDepth.cpp \
    ../../neolixai/imagepro/Getxy.cpp \
    ../../neolixai/imagepro/poprang.cpp \
    ../../neolixai/imagepro/Utils.cpp \
    ../../neolixai/measure/measure.cpp \
    ../../neolixai/pointcloud/ransac.cpp \
    ../../SampleCode.cpp \
    ../../neolixai/buildmodel/backgroundReconstruction.cpp \
    ../../neolixai/pointcloud/rotatePlane.cpp \
    ../../neolixai/test/test.cpp \
    ../../neolixai/measure/measure3D.cpp

HEADERS += \
    ../../include/libTof.h \
    ../../include/libusb.h \
    ../../neolixai/common/common.hpp \
    ../../neolixai/common/DepthRender.hpp \
    ../../neolixai/common/MatViewer.hpp \
    ../../neolixai/common/type.h \
    ../../neolixai/common/Utils.hpp \
    ../../neolixai/config/config.h \
    ../../neolixai/config/hardDebug.h \
    ../../neolixai/driverdecorate/base.h \
    ../../neolixai/driverdecorate/degug.h \
    ../../neolixai/driverdecorate/getfeatures.hpp \
    ../../neolixai/imagepro/CalDepth.h \
    ../../neolixai/imagepro/Utils.h \
    ../../neolixai/measure/measure.h \
    ../../neolixai/pointcloud/pointcloud.hpp \
    ../../neolixai/pointcloud/ransac.h \
    ../../neolixai/neolixMV.h \
    ../../neolixai/pointcloud/rotatePlane.h \
    ../../neolixai/test/test.h \
    ../../neolixai/measure/measure3D.h

LIBS += /usr/local/lib/libopencv_core.so \
        /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_calib3d.so \
        ../../lib/libTof.a
LIBS += -L"/usr/local/lib" -lusb-1.0 -lpthread


INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
               /usr/local/include/opencv2
