if(USB2AX)
   set(DXL_HAL_SRC ../dynamixelsdk/dxl_hal_usb2ax.C)
 else(USB2AX)
   set(DXL_HAL_SRC ../dynamixelsdk/dxl_hal_usb2dynamixel.C)
endif(USB2AX)

include_directories(../)

add_library(${PROJECT_NAME}_armparser
            generic_arm.cpp
            simple_arm.cpp
            ../dynamixelsdk/dynamixel.C
            ${DXL_HAL_SRC}
            )
