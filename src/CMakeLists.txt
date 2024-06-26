cmake_minimum_required(VERSION 3.16)

set(SOURCE
        "Core/BlackBoard.cpp"
        "Core/ChaosMonkey.cpp"
        "Core/Daemon.cpp"
        "Core/Logger.cpp"
        "Core/Config.cpp"
        "Core/PerformanceMonitor.cpp"
        "DataLink/AsyncSerial/AsyncSerial.cpp"
        "DataLink/AsyncSerial/BufferedAsyncSerial.cpp"
        "DataLink/HttpServer.cpp"
        "DataLink/InfantrySerialPort.cpp"
        "DataLink/HeroSerialPort.cpp"
        "DataLink/SentrySerialPort.cpp"
        "DataLink/AerialSerialPort.cpp"
        "DataLink/PosSynchronization.cpp"
        "Detect/NumClassifier.cpp"
        "Detect/ArmorDetector.cpp"
        "Detect/TensorRTDetector"
        "Detect/RadarNNetDetector.cpp"
        "Detect/RuneDetector.cpp"
        "HubMain.cpp"
        "Postprocess/CarPredictor.cpp"
        "Postprocess/PeriodPredictor.cpp"
        "Postprocess/EnergyPredictor.cpp"
        "Postprocess/PeriodOutpostPredictor.cpp"
        "Preprocess/ColorCalibration.cpp"
        "Preprocess/IMUFilter.cpp"
        "Preprocess/Undistort.cpp"
        "Sensor/DahengDriver.cpp"
        "Sensor/ImageSequenceReplay.cpp"
        "Sensor/VideoReplay.cpp"
        "Sensor/HikDriver.cpp"
        "Sensor/CameraBase.cpp"
        "Solve/AngleSolver.cpp"
        "Solve/PeriodSolver.cpp"
        "Solve/ArmorLocator.cpp"
        "Solve/RadarLocator.cpp"
        "Solve/BotLocator.cpp"
        "Strategy/Hero.cpp"
        "Strategy/Infantry.cpp"
        "Strategy/Sentry.cpp"
        "Test/ActorFramework/Actor.cpp"
        "Test/ActorFramework/Availability.cpp"
        "Test/ActorFramework/DaemonTester.cpp"
        "Test/ActorFramework/FrameRateKeeperTester.cpp"
        "Test/DataLink/JudgeSystemTester.cpp"
        #    "Test/Detect/FakeSerialPort.cpp"
        #    "Test/Sensor/FixedIMU.cpp"
        #    "Test/Solve/FakeDetector.cpp"
        #    "Test/Solve/SimpleStrategy.cpp"
        #    "Test/Solve/Simulator.cpp"
        "Test/TestHelper.cpp"
        "Utility/ArmorDetectorDrawer.cpp"
        "Utility/Crc.cpp"
        "Utility/EKF.cpp"
        "Utility/OpenCVHelper.cpp"
        "Utility/PID.cpp"
        "Utility/Timer.cpp"
        "Utility/VideoRecorder.cpp"
        "Utility/MathTools.cpp"
        "Utility/RadarInfo.cpp"
        "Utility/NetInference.cpp"
        "Utility/TRTModule.cpp"
        "Utility/ArmorLabeler.cpp"
        "Utility/RosCamera.cpp"
)

find_package(CAF REQUIRED COMPONENTS core io)
find_package(glm CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_path(CPP_HTTPLIB_INCLUDE_DIRS "httplib.h")
find_package(OpenCV CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(magic_enum CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)

add_executable(ArtinxHub ${SOURCE})
target_include_directories(ArtinxHub PRIVATE ${CAF_INCLUDE_DIRS})
target_link_libraries(ArtinxHub PRIVATE ceres Ceres::ceres)
target_link_libraries(ArtinxHub PRIVATE CAF::core)
target_link_libraries(
        ArtinxHub PRIVATE opencv_core opencv_videoio opencv_mcc opencv_calib3d
        opencv_tracking)#opencv_barcode
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    target_link_libraries(ArtinxHub PRIVATE Iphlpapi)
endif ()
target_link_libraries(ArtinxHub PRIVATE glm::glm)
target_include_directories(ArtinxHub PRIVATE ${CPP_HTTPLIB_INCLUDE_DIRS})

if (DEFINED ENV{DAHENG_SDK})
    if (${ARTINX_HUB_CAMERA} STREQUAL "USB3")
        set(DAHENG_SDK_PATH $ENV{DAHENG_SDK})
    elseif (${ARTINX_HUB_CAMERA} STREQUAL "USB2")
        set(DAHENG_SDK_PATH $ENV{DAHENG_SDK}/sdk)
    else ()
        message(FATAL "Invalid camera type")
    endif ()
    message(${ARTINX_HUB_CAMERA} ": DaHeng SDK found at " ${DAHENG_SDK_PATH})
    if (${ARTINX_HUB_CAMERA} STREQUAL "USB2")
        target_compile_definitions(ArtinxHub PRIVATE "ARTINX_DAHENG_USB2")
        target_include_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/include)
        target_link_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/lib/pub)
        target_link_directories(ArtinxHub PRIVATE
                ${DAHENG_SDK_PATH}/genicam/bin/Linux64_x64)
    else ()
        target_include_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/inc)
        if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
            target_link_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/lib/armv8)
        else ()
            target_link_directories(ArtinxHub PRIVATE ${DAHENG_SDK_PATH}/lib/x64)
        endif ()
    endif ()

    add_compile_definitions("ARTINX_DAHENG")
    target_link_libraries(ArtinxHub PRIVATE gxiapi)
endif ()

if (DEFINED ENV{HIK_SDK})
    target_include_directories(ArtinxHub PRIVATE $ENV{HIK_SDK}/include)
    if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
        target_link_directories(ArtinxHub PRIVATE $ENV{HIK_SDK}/lib/aarch64)
    else ()
        target_link_directories(ArtinxHub PRIVATE $ENV{HIK_SDK}/lib/64)
    endif ()
    target_link_libraries(
            ArtinxHub PRIVATE MVGigEVisionSDK FormatConversion MvCameraControl
            MediaProcess MVRender MvUsb3vTL)
    add_compile_definitions("ARTINX_HIK")
    message("USB3: Hik SDK found at $ENV{HIK_SDK}")
endif ()
target_link_libraries(ArtinxHub PRIVATE magic_enum::magic_enum)
target_link_libraries(ArtinxHub PRIVATE fmt::fmt)

if (DEFINED ENV{CUDA_PATH})
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    find_package(TensorRT REQUIRED)
    target_link_libraries(ArtinxHub PRIVATE $ENV{TensorRT_SOURCE} ${TensorRT_LIBS} cuda nvonnxparser)
    add_compile_definitions("ARTINX_CUDA")
endif()

if (DEFINED ENV{ROS_DISTRO})
    set(ROS_PATH /opt/ros/$ENV{ROS_DISTRO}/)
    set(DEPENDENCIES
            ament_cmake
            rclcpp
            std_msgs
            rclcpp_components
            sensor_msgs
            image_transport
            vision_opencv
            image_transport_plugins
    )
    foreach(package ${DEPENDENCIES})
        include_directories(ArtinxHub ${ROS_PATH}/include/${package})
        find_package(${package} REQUIRED)
        include_directories(ArtinxHub ${${package}_INCLUDE_DIRS})
        target_link_libraries(ArtinxHub PRIVATE ${${package}_LIBRARIES})
    endforeach()
    add_compile_definitions("ARTINX_ROS")
else()
    message("ROS2 not found")
endif ()

if (DEFINED ENV{ARTINX_RADAR})
    set(CUDA_TOOLKIT_ROOT_DIR $ENV{CUDA_PATH})
    find_package(CUDA REQUIRED)
    target_link_libraries(ArtinxHub PRIVATE ${CUDA_LIBRARIES})
    target_link_directories(ArtinxHub PRIVATE $ENV{TensorRT_PATH}/lib)
    target_link_libraries(ArtinxHub PRIVATE nvinfer)
    target_link_directories(ArtinxHub PRIVATE $ENV{TRT_INFER_PATH}/lib)
    target_link_libraries(ArtinxHub PRIVATE trtinfer)
endif ()

if (DEFINED ENV{OPENVINO2022_PATH})
    add_compile_definitions("ARTINX_OPENVINO2022")
    set(InferenceEngine_DIR $ENV{OPENVINO2022_PATH}/runtime/cmake)
    find_package(InferenceEngine CONFIG REQUIRED)
    target_link_libraries(ArtinxHub PRIVATE ${InferenceEngine_LIBRARIES})
endif ()

target_link_libraries(ArtinxHub PRIVATE nlohmann_json
        nlohmann_json::nlohmann_json)
target_link_libraries(ArtinxHub PRIVATE Eigen3::Eigen)
target_link_libraries(ArtinxHub PRIVATE spdlog::spdlog
        spdlog::spdlog_header_only)

add_custom_command(
        TARGET ArtinxHub
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/pages
        $<TARGET_FILE_DIR:ArtinxHub>/pages)
add_custom_command(
        TARGET ArtinxHub
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/scripts
        $<TARGET_FILE_DIR:ArtinxHub>/scripts)
add_custom_command(
        TARGET ArtinxHub
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/data $<TARGET_FILE_DIR:ArtinxHub>/data)

add_custom_command(
        TARGET ArtinxHub
        PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/record $<TARGET_FILE_DIR:ArtinxHub>/record)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    target_compile_options(ArtinxHub PRIVATE /W4 /WX-)
    add_custom_command(
            TARGET ArtinxHub
            PRE_BUILD
            COMMAND
            ${CMAKE_COMMAND} -E copy ${DAHENG_SDK_PATH}/../../APIDll/Win64/GxIAPI.dll
            $<TARGET_FILE_DIR:ArtinxHub>)
    set(INFERENCE_ENGINE_ROOT
            $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/inference_engine
    )
    file(GLOB INFERENCE_ENGINE_DLLS
            ${INFERENCE_ENGINE_ROOT}/bin/intel64/${CMAKE_BUILD_TYPE}/*.dll)
    add_custom_command(
            TARGET ArtinxHub
            PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy ${INFERENCE_ENGINE_DLLS}
            $<TARGET_FILE_DIR:ArtinxHub>)
    file(GLOB_RECURSE INFERENCE_ENGINE_EXTERNAL_DLLS
            ${INFERENCE_ENGINE_ROOT}/external/*.dll)
    add_custom_command(
            TARGET ArtinxHub
            PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy ${INFERENCE_ENGINE_EXTERNAL_DLLS}
            $<TARGET_FILE_DIR:ArtinxHub>)
    file(
            GLOB NGRAPH_DLLS
            $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/ngraph/lib/*.dll)
    list(
            REMOVE_ITEM
            NGRAPH_DLLS
            $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/ngraph/lib/libprotobuf.dll
    )
    list(
            REMOVE_ITEM
            NGRAPH_DLLS
            $ENV{ONEAPI_ROOT}/openvino/openvino_2021/deployment_tools/ngraph/lib/libprotobufd.dll
    )
    add_custom_command(
            TARGET ArtinxHub
            PRE_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy ${NGRAPH_DLLS}
            $<TARGET_FILE_DIR:ArtinxHub>)
    add_custom_command(
            TARGET ArtinxHub
            PRE_BUILD
            COMMAND
            ${CMAKE_COMMAND} -E copy
            ${INFERENCE_ENGINE_ROOT}/bin/intel64/${CMAKE_BUILD_TYPE}/plugins.xml
            $<TARGET_FILE_DIR:ArtinxHub>)
endif ()

add_custom_command(TARGET ArtinxHub PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/config_templates/caf-application.conf $<TARGET_FILE_DIR:ArtinxHub>)
