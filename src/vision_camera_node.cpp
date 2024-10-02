#include "../include/vision_camera/vision_camera_node.hpp"

namespace vision
{
    UsbCamera::UsbCamera(const rclcpp::NodeOptions &options) : Node("usb_camera", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting usb_camera Node!");
        cap_ = std::make_shared<cv::VideoCapture>();
        // Load camera info
        camera_dev_ = this->declare_parameter("camera_dev", "/dev/video0");
        RCLCPP_INFO(this->get_logger(), "Opening %s", camera_dev_.c_str());
        camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
        camera_info_manager_ =
            std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
        auto camera_info_url =
            this->declare_parameter("camera_info_url", "package://vision_camera/config/camera_info.yaml");
        if (camera_info_manager_->validateURL(camera_info_url))
        {
            camera_info_manager_->loadCameraInfo(camera_info_url);
            camera_info_msg_ = camera_info_manager_->getCameraInfo();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
        }

        // set image_msg info
        this->image_msg_.set__encoding("bgr8");
        this->image_msg_.header.set__frame_id("camera_optical_frame");
        this->image_msg_.set__width(camera_info_msg_.width);
        this->image_msg_.set__height(camera_info_msg_.height);
        this->image_msg_.set__step(camera_info_msg_.width * 3);

        // declare parameters
        bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
        auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
        camera_pub_ = image_transport::create_camera_publisher(this, camera_name_+"/image_raw", qos);
        declareParameters();
        params_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&UsbCamera::parametersCallback, this, std::placeholders::_1));

        // set 1hz watchdog
        using namespace std::chrono_literals;
        watchdog_timer_ = create_wall_timer(1s, std::bind(&UsbCamera::watchdog_callback, this));
    }

    UsbCamera::~UsbCamera()
    {
        if (cam_thread_.joinable())
        {
            cam_thread_.join();
            cap_->release();
        }
    }

    void UsbCamera::declareParameters()
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.integer_range.resize(1);
        param_desc.integer_range[0].step = 1;
        // Exposure time
        param_desc.description = "Exposure time in microseconds";
        param_desc.integer_range[0].from_value = 100;
        param_desc.integer_range[0].to_value = 20000;
        auto exposure_time = this->declare_parameter("exposure_time", 1000, param_desc);
        RCLCPP_INFO(this->get_logger(), "Exposure time: %ld", exposure_time);

        // Gain
        param_desc.description = "Gain";
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 50;
        auto gain = this->declare_parameter("gain", 0, param_desc);
        RCLCPP_INFO(this->get_logger(), "Gain: %ld", gain);

        // FPS
        param_desc.description = "FPS";
        param_desc.integer_range[0].from_value = 30;
        param_desc.integer_range[0].to_value = 120;
        auto fps = this->declare_parameter("fps", 30, param_desc);
        RCLCPP_INFO(this->get_logger(), "fps: %ld", fps);
    }

    rcl_interfaces::msg::SetParametersResult UsbCamera::parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : parameters)
        {
            if (param.get_name() == "exposure_time")
            {
                cap_->set(cv::CAP_PROP_EXPOSURE, (double)param.as_int());
            }
            else if (param.get_name() == "gain")
            {
                cap_->set(cv::CAP_PROP_GAIN, (double)param.as_int());
            }
            else
            {
                result.successful = false;
                result.reason = "Unknown parameter: " + param.get_name();
            }
        }
        return result;
    }

    void UsbCamera::watchdog_callback()
    {
        if (cap_->isOpened())
        {
            return;
        }
        else
        {
            cap_ = std::make_shared<cv::VideoCapture>(camera_dev_, cv::CAP_V4L);
            cap_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap_->set(cv::CAP_PROP_FRAME_WIDTH, camera_info_msg_.width);
            cap_->set(cv::CAP_PROP_FRAME_HEIGHT, camera_info_msg_.height);
            cap_->set(cv::CAP_PROP_FPS, (double)this->get_parameter("fps").as_int());
            cap_->set(cv::CAP_PROP_AUTO_EXPOSURE, 1); // 1为手动曝光，3为自动曝光
            cap_->set(cv::CAP_PROP_EXPOSURE, (double)this->get_parameter("exposure_time").as_int());
            cap_->set(cv::CAP_PROP_GAIN, (double)this->get_parameter("gain").as_int());
            cap_->set(cv::CAP_PROP_AUTO_WB, 1);
            cam_thread_ = std::thread{
                [this]() -> void
                {
                    while (rclcpp::ok())
                    {
                        cap_->read(src_);
                        auto format = src_.type() == CV_8UC1 ? "mono8" : "bgr8";
                        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), format, src_).toImageMsg();
                        image_msg_.set__data(msg->data);
                        image_msg_.header.set__stamp(now());
                        camera_info_msg_.header = image_msg_.header;
                        camera_pub_.publish(image_msg_, camera_info_msg_);
                    }
                }};
            cam_thread_.detach();
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(vision::UsbCamera)