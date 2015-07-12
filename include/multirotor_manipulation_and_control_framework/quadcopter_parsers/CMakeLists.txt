# Add Quadcopter Parser Library

if (ENABLE_PIXHAWK)
  set(PARSER_SRCS ${PARSER_SRCS} pixhawk_parser.cpp)
endif (ENABLE_PIXHAWK)

# Keep adding if more present
if (DEFINED PARSER_SRCS)
  set(PARSER_LIBRARY_DEFINED ON PARENT_SCOPE)
  add_library(${PROJECT_NAME}_quadcopterparser
              ${PARSER_SRCS}
             )
endif (DEFINED PARSER_SRCS)
