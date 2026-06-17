cmake_minimum_required(VERSION 3.22)

set(DEVICE_MODULE_DIRS motors imu led)

set(DEVICE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/motors/src/motor.cpp)

foreach(module ${DEVICE_MODULE_DIRS})
    if(module STREQUAL "motors")
        foreach(sub djimotors lkmotors dmmotors)
            file(GLOB sub_sources ${CMAKE_CURRENT_SOURCE_DIR}/motors/${sub}/src/*.cpp)
            list(APPEND DEVICE_SOURCES ${sub_sources})
        endforeach()
    else()
        file(GLOB module_sources ${CMAKE_CURRENT_SOURCE_DIR}/${module}/src/*.cpp)
        list(APPEND DEVICE_SOURCES ${module_sources})
    endif()
endforeach()

add_library(pnx_devices STATIC ${DEVICE_SOURCES})

set(DEVICE_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/motors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/motors/djimotors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/motors/lkmotors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/motors/dmmotors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/imu/include
    ${CMAKE_CURRENT_SOURCE_DIR}/led/include
)

target_include_directories(pnx_devices PUBLIC ${DEVICE_INCLUDES})

target_link_libraries(pnx_devices PUBLIC pnx_bsp pnx_libraries pnx_os pnx_platform)

target_compile_features(pnx_devices PUBLIC cxx_std_17)
