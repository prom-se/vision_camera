#include "../include/vision_camera/vision_camera_node.hpp"

Camera_node::Camera_node(std::string type,uint8_t index):rclcpp::Node("vision_camera"),
handle{nullptr},
isOk{false},
camType{type},camIndex{index},
width{1280},height{1024},pixelFormat{PixelType_Gvsp_BayerGR8},expFrameRate{120},
exposureTime{3000},gain{5.0},
src{new cv::Mat(height,width,CV_8UC3)}
{
    //设置重启计时器1hz.
    reopenTimer=create_wall_timer(
    1s, std::bind(&Camera_node::cameraReopen_callback, this));

    robotSub=create_subscription<vision_interfaces::msg::Robot>(
        "/serial_driver/robot",rclcpp::SensorDataQoS(),std::bind(&Camera_node::robot_callback,this,std::placeholders::_1));
    robotPtr=std::make_unique<vision_interfaces::msg::Robot>();

    tfBroadcaster=std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    cameraInfoManager=std::make_unique<camera_info_manager::CameraInfoManager>(this, "Hik_03");
    auto camera_info_url =this->declare_parameter("camera_info_url", "package://vision_camera/config/hik_03_config.yaml");
    cameraInfoManager->loadCameraInfo(camera_info_url);

    camPub= create_publisher<sensor_msgs::msg::Image>("/vision_camera/image_raw",rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)));
    infoPub= create_publisher<sensor_msgs::msg::CameraInfo>("/vision_camera/camera_info",rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)));

    isOk = camType=="hik"?initHikCam():initUsbCam(camType);
}

Camera_node::~Camera_node(){

}

bool Camera_node::initHikCam(){ 
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if(stDeviceList.nDeviceNum!=0){
        MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[camIndex]);
        nRet = MV_CC_OpenDevice(handle);
    }
    else{
        RCLCPP_FATAL(get_logger(),"未找到海康相机！");
        return false;
    }
    if(nRet==MV_OK){
        RCLCPP_INFO(get_logger(),"成功打开海康相机[%d]!",camIndex);
        MV_CC_SetIntValue(handle, "Width", width);
        MV_CC_SetIntValue(handle, "Height", height);
        MV_CC_SetPixelFormat(handle, pixelFormat);
        MV_CC_SetFloatValue(handle, "ExposureTime", exposureTime);
        MV_CC_SetFloatValue(handle, "Gain", gain);
        MV_CC_SetFrameRate(handle,expFrameRate);
        MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
        MV_CC_SetEnumValue(handle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        MV_CC_RegisterImageCallBackEx(handle,&Camera_node::hikImgCallback,this);
        MV_CC_StartGrabbing(handle);
        return true;
    }
    else{
        RCLCPP_ERROR(get_logger(),"打开海康相机[%d]失败!",camIndex);
        return false;
    }
}

bool Camera_node::initUsbCam(std::string type){
    if(type=="usb"){
        usbCam.open(camIndex);
    }
    else if(type=="video"){
        usbCam.open("/home/promise/video/aprilTag.mp4");
    }
    else{
        RCLCPP_FATAL(get_logger(),"错误摄像头类型!");
        return false;
    }
    bool ret = usbCam.isOpened();
    if(ret){
        std::thread{[this]() -> void {
            int frameCounter = 0;
            while(rclcpp::ok()){
                std::this_thread::sleep_for(std::chrono::milliseconds(1000/60));
                usbCam >> *src;
                frameCounter += 1;
                if (frameCounter == int(usbCam.get(cv::CAP_PROP_FRAME_COUNT))){
                    frameCounter = 0;
                    usbCam.set(cv::CAP_PROP_POS_FRAMES, 0);
                }
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", *this->src).toImageMsg();

                sensor_msgs::msg::CameraInfo info;
                info=cameraInfoManager->getCameraInfo();
                info.header=msg->header;
                this->camPub->publish(*msg);
            }
        }}.detach();
        RCLCPP_INFO(get_logger(),"打开USB摄像头成功!");
        return true;
    }
    else{
        RCLCPP_FATAL(get_logger(),"打开USB摄像头失败!");
        return false;
    }
}

void Camera_node::cameraReopen_callback(){
    if(!isOk){
        RCLCPP_WARN(get_logger(),"重启摄像头...");
        isOk = camType=="hik"?initHikCam():initUsbCam(camType);
    }
}

void Camera_node::hikImgCallback(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser){
    const Camera_node *thisPtr = (Camera_node*)pUser;
    cv::Mat src = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1);
    memcpy(src.data, pData, pFrameInfo->nWidth * pFrameInfo->nHeight * 1);
    cv::cvtColor(src, *thisPtr->src, cv::COLOR_BayerGR2BGR_EA);
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", *thisPtr->src).toImageMsg();

    sensor_msgs::msg::CameraInfo info;
    info=thisPtr->cameraInfoManager->getCameraInfo();
    info.header=msg->header;
    thisPtr->camPub->publish(*msg);
    thisPtr->infoPub->publish(info);
}

void Camera_node::robot_callback(const vision_interfaces::msg::Robot robot){
    try{
        *robotPtr=robot;
    }
    catch(std::system_error &error){
        RCLCPP_ERROR(get_logger(),"获取机器人信息时发生错误.");
    }
    tf2::Quaternion q;
    q.setRPY(0,robotPtr->self_pitch*M_PI/180.0,robotPtr->self_yaw*M_PI/180.0);
    q.normalize();
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "camera";
    t.transform.rotation.x=q.x();
    t.transform.rotation.y=q.y();
    t.transform.rotation.z=q.z();
    t.transform.rotation.w=q.w();
    tfBroadcaster->sendTransform(t);
}

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);

    std::string camType=argc>1?argv[1]:"hik";
    uint8_t camIndex = std::stoi(argc>2?argv[2]:"0");
    auto node = std::make_shared<Camera_node>(camType,camIndex);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
