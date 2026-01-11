target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/bsp_current.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/bsp_voltage.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/bsp_temp.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/bsp_motor.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/bsp_imu.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/bsp_current.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/bsp_voltage.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/bsp_temp.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/bsp_motor.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/bsp_imu.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)