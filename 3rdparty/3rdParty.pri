# 引入opencv
#INCLUDEPATH += D:/apps/opencv/build/include/
#INCLUDEPATH += D:/apps/opencv/build/include/opencv2/
#LIBS += -LD:/apps/opencv/build/x64/vc15/lib/ -lopencv_world440

# 引入Eigen矩阵运算库
#INCLUDEPATH += D:\apps\eigen-git-mirror-master

# 引入PAGMO2库
include(./PAGMO2/PAGMO2.pri)

# 引入PCL点云库
include(./PCL191/PCL191.pri)

# 引入OCC库
include(./OCC77/OCC77.pri)

# 引入FCL库
include(./FCL/FCL.pri)

# 引入OMPL库
include(./OMPL/OMPL.pri)
