cmake_minimum_required (VERSION 3.19)

set(SOURCE Core/Daemon.cpp
Core/PerformanceMonitor.cpp
DataLink/HttpServer.cpp
Sensor/DahengDriver.cpp
Sensor/VideoReplay.cpp
Test/Test.cpp
Utility/VideoRecoder.cpp
HubMain.cpp
)

find_package(CAF REQUIRED COMPONENTS core io)
find_package(glm CONFIG REQUIRED)
find_package(simdjson CONFIG REQUIRED)
find_path(CPP_HTTPLIB_INCLUDE_DIRS "httplib.h")
find_package(OpenCV CONFIG REQUIRED)
set(DAHENG_SDK_PATH $ENV{DAHENG_SDK})
message(${DAHENG_SDK_PATH})

add_executable(ArtinxHub ${SOURCE})
target_include_directories(ArtinxHub PRIVATE ${CAF_INCLUDE_DIRS})
target_link_libraries(ArtinxHub PRIVATE CAF::core)
target_link_libraries(ArtinxHub PRIVATE opencv_core opencv_videoio)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_link_libraries(ArtinxHub PRIVATE Iphlpapi)
endif()
target_link_libraries(ArtinxHub PRIVATE glm::glm)
target_include_directories(ArtinxHub PRIVATE ${CPP_HTTPLIB_INCLUDE_DIRS})
target_include_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/inc)
target_link_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/lib/x64)
target_link_libraries(ArtinxHub PRIVATE gxiapi)

add_custom_command(TARGET ArtinxHub PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/pages $<TARGET_FILE_DIR:ArtinxHub>/pages)
add_custom_command(TARGET ArtinxHub PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/scripts $<TARGET_FILE_DIR:ArtinxHub>/scripts)

file(COPY ${DAHENG_SDK_PATH}/../../APIDll/Win64/GxIAPI.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
