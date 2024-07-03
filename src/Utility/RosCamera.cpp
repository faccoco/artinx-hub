//
// Created by 12012710 on 24-6-23.
//
#ifdef ARTINX_ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <caf/event_based_actor.hpp>

#include "BlackBoard.hpp"
#include "CameraFrame.hpp"
#include "ExceptionProbe.hpp"
#include "Hub.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>

struct RosCameraSettings final {
    std::string ip;
    std::string imageTopic;
    std::string cameraInfoTopic;
    std::string jointTopic;
};

template <class Inspector>
bool inspect(Inspector& f, RosCameraSettings& x) {
    return f.object(x).fields(f.field("ip", x.ip).fallback("127.0.0.1"), f.field("imageTopic", x.imageTopic).fallback("/image"),
                              f.field("cameraInfoTopic", x.cameraInfoTopic).fallback("/camera_info"),
                              f.field("jointTopic", x.jointTopic).fallback("/joint"));
}
//
class RosCamera final : public HubHelper<caf::event_based_actor, RosCameraSettings, image_frame_atom> {
    Identifier mKey;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSubscription;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscription;
    rclcpp::Node::SharedPtr node;
    cv::Mat mCameraMatrix;
    cv::Mat mDistCoefficients;
    cv::Point2f camCenter;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> camerainfo;

    double yaw;
    double pitch;

public:
    RosCamera(caf::actor_config& base, const HubConfig& config, std::string name)
        : HubHelper{ base, config, std::move(name) }, mKey{ generateKey(this) } {
        yaw = 0;
        pitch = 0;
        rclcpp::init(0, nullptr);  // init ROS
        // initialize node
        node = rclcpp::Node::make_shared("ros_connector");
        cameraInfoSubscription = node->create_subscription<sensor_msgs::msg::CameraInfo>(
            mConfig.cameraInfoTopic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg) {
                camCenter = cv::Point2f(cameraInfoMsg->k[2], cameraInfoMsg->k[5]);
                camerainfo = std::make_shared<sensor_msgs::msg::CameraInfo>(*cameraInfoMsg);
                auto cameraMatrix = camerainfo->k;
                auto distCoefficients = camerainfo->d;
                mCameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(camerainfo->k.data())).clone();
                mDistCoefficients = cv::Mat(1, 5, CV_64F, const_cast<double*>(camerainfo->d.data())).clone();
                cameraInfoSubscription.reset();
            });
        imgSubscription = node->create_subscription<sensor_msgs::msg::Image>(
            mConfig.imageTopic, rclcpp::SensorDataQoS(), std::bind(&RosCamera::publishRosImg, this, std::placeholders::_1));
        jointSubscription = node->create_subscription<sensor_msgs::msg::JointState>(
            mConfig.jointTopic, rclcpp::SensorDataQoS(), std::bind(&RosCamera::publishJointState, this, std::placeholders::_1));
        std::thread([this]() { rclcpp::spin(node); }).detach();
    }

    void publishRosImg(const sensor_msgs::msg::Image::ConstSharedPtr& imgMsg) {
        ACTOR_EXCEPTION_PROBE();
        // const auto frame = cv_bridge::toCvShare(imgMsg, "rgb8")->image;
        // const auto frame = cv_bridge::toCvShare(imgMsg, "rgb8")->image;
        // cv::Mat frame;
        const auto frame = convertToCvMat(imgMsg);
        CameraFrame frameData;
        frameData.lastUpdate = SynchronizedClock::instance().now();
        frameData.info.cameraMatrix = mCameraMatrix;
        frameData.info.distCoefficients = mDistCoefficients;
        frameData.info.identifier = "RosCamera";
        frameData.info.width = frame.cols;
        frameData.info.height = frame.rows;
        frameData.info.tfRobot2Camera = clcTfRobot2Camera(yaw, pitch, 0.0);
        frame.copyTo(frameData.frame);

        if (frameData.info.cameraMatrix.empty()){
            return ;
        }
        sendAll(image_frame_atom_v,
                BlackBoard::instance().updateSync(mKey, std::move(frameData), static_cast<std::string_view>("ros_camera")));
    }

    void publishJointState(const sensor_msgs::msg::JointState::ConstSharedPtr& jointMsg){
        ACTOR_EXCEPTION_PROBE();
        yaw = jointMsg->position[0];
        pitch = jointMsg->position[1];
        HubLogger::watch("simYaw", yaw);
        HubLogger::watch("simPitch", pitch);
    }

    Transform<FrameOfRef::Robot, FrameOfRef::Camera, true> clcTfRobot2Camera(const double yawAngle, const double pitchAngle,
                                                                             const double rollAngle) {
        return glm::rotate(
            glm::rotate(glm::rotate(glm::identity<glm::dmat4>(), -rollAngle, glm::dvec3{ 0, 0, 1 }), -pitchAngle, glm::dvec3{ 1, 0, 0 }),
            -yawAngle, glm::dvec3{ 0, 1, 0 });
    }

    caf::behavior make_behavior() override {
        return { [this](start_atom) { ACTOR_PROTOCOL_CHECK(start_atom); } };
    };

    static cv::Mat convertToCvMat(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        // Convert the ROS Image message to OpenCV image
        int cv_type;
        if (msg->encoding == "mono8")
        {
            cv_type = CV_8UC1;
        }
        else if (msg->encoding == "bgr8" || msg->encoding == "rgb8")
        {
            cv_type = CV_8UC3;
        }
        else if (msg->encoding == "mono16")
        {
            cv_type = CV_16UC1;
        }
        else if (msg->encoding == "rgba8")
        {
            cv_type = CV_8UC4;
        }
        else
        {
            logInfo(fmt::format("Unsupported encoding type: {}", msg->encoding));
            return {};
        }

        cv::Mat mat(msg->height, msg->width, cv_type, const_cast<unsigned char*>(msg->data.data()), msg->step);

        if (msg->encoding == "rgb8")
        {
            cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
        }
        return mat.clone(); // Ensure data is copied
    }
};

HUB_REGISTER_CLASS(RosCamera);
#endif