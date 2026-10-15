# Add sources to executable/library
target_sources(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/syscall.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/sysmem.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/startup_stm32g474xx.S"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/ADC/adc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/G474RE_Drivers/uart.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/Kobuki_Drivers/kobuki.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/Ultrasonic/ultrasonic.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/Pathfind/pathfind.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/Navigation/navigation.c"

)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/stm32g474xe_flash.ld" "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/stm32g474xe_flash.ld")
