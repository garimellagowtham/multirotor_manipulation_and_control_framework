# Build controller
include_directories(../quadcopter_parsers)
add_library(${PROJECT_NAME}_quadcoptercontroller
    trajectory_tracking_control.cpp
    )
