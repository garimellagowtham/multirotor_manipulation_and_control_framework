#Add subdirectories
message("Parsing include/multirotor_manipulation_and_control_framework folder")

add_subdirectory(arm_controllers)  
add_subdirectory(arm_parsers)        
add_subdirectory(filters)
add_subdirectory(quadcopter_controllers)
add_subdirectory(quadcopter_parsers)

# Create a list of libraries being created:

set(PROJECT_LIBS ${PROJECT_NAME}_armparser  ${PROJECT_NAME}_stateestimator ${PROJECT_NAME}_quadcoptercontroller)

if(PARSER_LIBRARY_DEFINED)
  set(PROJECT_LIBS ${PROJECT_LIBS} ${PROJECT_NAME}_quadcopterparser)
endif(PARSER_LIBRARY_DEFINED)
