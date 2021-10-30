cmake_minimum_required (VERSION 3.19)

set(SOURCE 
"Core/BlackBoard.cpp"
"Core/Daemon.cpp"
"Core/PerformanceMonitor.cpp"
"DataLink/HttpServer.cpp"
"Sensor/DahengDriver.cpp"
"Sensor/VideoReplay.cpp"
"Detect/ArmorDetector.cpp"
"Detect/CarDetector.cpp"
"Preprocess/ColorCalibration.cpp"
"Preprocess/IMUFilter.cpp"
"Preprocess/Undistort.cpp"
"Solve/AngleSolver.cpp"
"Solve/ArmorLocator.cpp"
"Test/ActorFramework/Test.cpp"
"Test/Detect/ArmorDetectorTester.cpp"
"Test/Solve/ArmorLocatorTester.cpp"
"Test/Solve/FakeDetector.cpp"
"Test/Solve/FakeHead.cpp"
"Test/Solve/FakeIMU.cpp"
"Test/Solve/SimpleStrategy.cpp"
"Test/Solve/Simulator.cpp"
"Utility/CarDetectorDrawer.cpp"
"Utility/OpenCVHelper.cpp"
"Utility/PID.cpp"
"Utility/Timer.cpp"
"Utility/VideoRecoder.cpp"
"HubMain.cpp")

find_package(CAF REQUIRED COMPONENTS core io)
find_package(glm CONFIG REQUIRED)
find_package(simdjson CONFIG REQUIRED)
find_path(CPP_HTTPLIB_INCLUDE_DIRS "httplib.h")
find_package(OpenCV CONFIG REQUIRED)
set(DAHENG_SDK_PATH $ENV{DAHENG_SDK})
message(${DAHENG_SDK_PATH})
find_package(Bullet CONFIG REQUIRED)
find_package(magic_enum CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)

set(InferenceEngine_DIR $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/inference_engine/share/)
find_package(InferenceEngine CONFIG REQUIRED)

add_executable(ArtinxHub ${SOURCE})
target_include_directories(ArtinxHub PRIVATE ${InferenceEngine_INCLUDE_DIRS})
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
target_link_libraries(ArtinxHub PRIVATE LinearMath Bullet3Common BulletDynamics BulletCollision)
target_link_libraries(ArtinxHub PRIVATE magic_enum::magic_enum)
target_link_libraries(ArtinxHub PRIVATE fmt::fmt)
target_link_libraries(ArtinxHub PRIVATE ${InferenceEngine_LIBRARIES})

add_custom_command(TARGET ArtinxHub PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/pages $<TARGET_FILE_DIR:ArtinxHub>/pages)
add_custom_command(TARGET ArtinxHub PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/scripts $<TARGET_FILE_DIR:ArtinxHub>/scripts)
add_custom_command(TARGET ArtinxHub PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/data $<TARGET_FILE_DIR:ArtinxHub>/data)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
file(COPY ${DAHENG_SDK_PATH}/../../APIDll/Win64/GxIAPI.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
set(INFERENCE_ENGINE_ROOT $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/inference_engine)
file(GLOB INFERENCE_ENGINE_DLLS ${INFERENCE_ENGINE_ROOT}/bin/intel64/${CMAKE_BUILD_TYPE}/*.dll)
file(COPY ${INFERENCE_ENGINE_DLLS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
file(GLOB_RECURSE INFERENCE_ENGINE_EXTERNAL_DLLS ${INFERENCE_ENGINE_ROOT}/external/*.dll)
file(COPY ${INFERENCE_ENGINE_EXTERNAL_DLLS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
file(GLOB NGRAPH_DLLS $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/ngraph/lib/*.dll)
file(COPY ${NGRAPH_DLLS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
file(COPY ${INFERENCE_ENGINE_ROOT}/bin/intel64/${CMAKE_BUILD_TYPE}/plugins.xml DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
endif()

file(COPY  ${CMAKE_SOURCE_DIR}/config_templates/caf-application.conf DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
