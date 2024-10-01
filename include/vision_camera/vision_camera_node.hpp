#ifndef VISION_CAMERA_NODE_HPP
#define VISION_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace vision
{
    class UsbCamera : public rclcpp::Node
    {
    public:
        explicit UsbCamera(const rclcpp::NodeOptions &options);
        ~UsbCamera();

    private:
        void watchdog_callback();
        void declareParameters();
        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters);

        std::thread cam_thread_;
        cv::Mat src_;
        std::shared_ptr<cv::VideoCapture> cap_;
        sensor_msgs::msg::Image image_msg_;
        image_transport::CameraPublisher camera_pub_;
        rclcpp::TimerBase::SharedPtr watchdog_timer_;
        std::string camera_dev_;
        std::string camera_name_;
        std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
        sensor_msgs::msg::CameraInfo camera_info_msg_;
        OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    };
}
#endif // VISION_CAMERA_NODE_HPP