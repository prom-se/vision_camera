#ifndef VISION_CAMERA_NODE_HPP
#define VISION_CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vision_interfaces/msg/robot.hpp>
#include <string>
#include <cmath>
#include <MvCameraControl.h>
#include <CameraParams.h>
#include <MvErrorDefine.h>
#include <PixelType.h>
#include <chrono>

using namespace std::chrono_literals;

class Camera_node : public rclcpp::Node{
public:
    explicit Camera_node(std::string type,uint8_t index);
    ~Camera_node();

    cv::Mat getImg();
private:
    bool initHikCam();
    bool initUsbCam(std::string type);
    static void hikImgCallback(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
    void cameraReopen_callback();
    void robot_callback(const vision_interfaces::msg::Robot robot);

    cv::VideoCapture usbCam;
    void* handle;
    bool isOk;
    std::string camType;
    const uint8_t camIndex;

    int width;
    int height;
    long pixelFormat;

    float expFrameRate;
    float exposureTime;
    float gain;
    
    std::unique_ptr<vision_interfaces::msg::Robot> robotPtr;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::TimerBase::SharedPtr reopenTimer;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camPub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub;

    std::unique_ptr<camera_info_manager::CameraInfoManager> cameraInfoManager;

    rclcpp::Subscription<vision_interfaces::msg::Robot>::SharedPtr robotSub;
    std::shared_ptr<cv::Mat> src;
};

#endif//VISION_CAMERA_NODE_HPP