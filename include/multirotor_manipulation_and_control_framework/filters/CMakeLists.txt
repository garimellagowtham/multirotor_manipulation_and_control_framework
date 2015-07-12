# Build Estimator library
include_directories(../quadcopter_parsers)
add_library(${PROJECT_NAME}_stateestimator
    point3d_estimator.cpp
    )
