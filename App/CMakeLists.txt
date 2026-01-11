target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/app_main.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/app_motor_controller.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/app_pid.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/app_retarget.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/app_utils.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/app_task.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc/app_chassis_controller.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/app_main.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/app_motor_controller.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/app_pid.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/app_retarget.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/app_utils.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/app_task.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/app_chassis_controller.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)