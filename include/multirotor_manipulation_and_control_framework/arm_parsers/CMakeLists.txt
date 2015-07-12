if(USB2AX)
   set(DXL_HAL_SRC ../extern/dynamixelsdk/dxl_hal_usb2ax.C)
 else(USB2AX)
   set(DXL_HAL_SRC ../extern/dynamixelsdk/dxl_hal_usb2dynamixel.C)
endif(USB2AX)

add_library(${PROJECT_NAME}_armparser
            generic_arm.cpp
            simple_arm.cpp
            ../extern/dynamixelsdk/dynamixel.C
            ${DXL_HAL_SRC}
            )
