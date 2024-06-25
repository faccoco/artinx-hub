// //
// // Created by 12012710 on 24-6-23.
// //
//
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <caf/event_based_actor.hpp>

#include "BlackBoard.hpp"
#include "Hub.hpp"

#include "CameraBase.hpp"
#include <CameraFrame.hpp>
#include <ExceptionProbe.hpp>

struct RosConnectorSettings final {
    std::string ip;
};

template <class Inspector>
bool inspect(Inspector& f, RosConnectorSettings& x) {
    return f.object(x).fields(f.field("ip", x.ip).fallback("127.0.0.1"));
}
//
class RosCameraDriver final : public CameraBase {
    Identifier mKey;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSubscription;
    rclcpp::Node::SharedPtr node;

    cv::Point2f camCenter;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> camerainfo;

public:
    RosCameraDriver(caf::actor_config& base, const HubConfig& config, std::string name)
        : CameraBase{ base, config, std::move(name) }, mKey{ generateKey(this) } {
        // initialize node
        node = rclcpp::Node::make_shared("ros_connector");
        cameraInfoSubscription = node->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg) {
                camCenter = cv::Point2f(cameraInfoMsg->k[2], cameraInfoMsg->k[5]);
                camerainfo = std::make_shared<sensor_msgs::msg::CameraInfo>(*cameraInfoMsg);
                mCameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(camerainfo->k.data())).clone();
                mDistCoefficients = cv::Mat(1, 5, CV_64F, const_cast<double*>(camerainfo->d.data())).clone();
                cameraInfoSubscription.reset();
            });
        imgSubscription = node->create_subscription<sensor_msgs::msg::Image>(
            "/image", rclcpp::SensorDataQoS(), std::bind(&publisRosImg, this, std::placeholders::_1));

        rclcpp::init(0, nullptr);
        std::thread([this]() { rclcpp::spin(node); }).detach();
    }

    void publisRosImg(const sensor_msgs::msg::Image::ConstSharedPtr& imgMsg) {
        auto frame = cv_bridge::toCvShare(imgMsg, "rgb8")->image;

        CameraFrame frameData;
        frameData.lastUpdate = SynchronizedClock::instance().now();
        frameData.info.cameraMatrix = mCameraMatrix;
        frameData.info.distCoefficients = mDistCoefficients;
        frameData.info.identifier = "RosCamera";
        frameData.info.width = frame.cols;
        frameData.info.height = frame.rows;
        frameData.info.tfRobot2Camera = clcTfRobot2Camera(Pose{ 0.0, 0.0, 0.0 });
        frame.copyTo(frameData.frame);

        sendAll(image_frame_atom_v,
                BlackBoard::instance().updateSync(mKey, std::move(frameData), static_cast<std::string_view>(mConfig.cameraName)));
    }
};
HUB_REGISTER_CLASS(RosCameraDriver);
