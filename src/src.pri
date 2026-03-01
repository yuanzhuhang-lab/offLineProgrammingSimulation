# Use Precompiled headers (PCH)
CONFIG += precompile_header
PRECOMPILED_HEADER = src/stable.h

HEADERS += \
    src/stable.h

SOURCES += \
    src/main.cpp

include( ./ui/ui.pri)
include( ./pose_estimation/pose_estimation.pri)
include( ./model_processing/model_processing.pri)
include( ./collision_detection/collision_detection.pri)
include( ./path_planning/path_planning.pri)
include( ./robot_config/robot_config.pri)


DISTFILES +=
