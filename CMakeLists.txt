cmake_minimum_required(VERSION 3.8)
project(vision_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV 4.8.0 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/opt/MVS/include/")

add_executable(${PROJECT_NAME}_node 
src/vision_camera_node.cpp)

target_include_directories(${PROJECT_NAME}_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS})

# 包含海康相机动态链接库
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  find_library(MVS_LIB libMvCameraControl.so /opt/MVS/lib/64/)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") 
  find_library(MVS_LIB libMvCameraControl.so /opt/MVS/lib/aarch64/)
endif()
target_link_libraries(${PROJECT_NAME}_node ${MVS_LIB})

ament_target_dependencies(${PROJECT_NAME}_node rclcpp cv_bridge geometry_msgs vision_interfaces sensor_msgs std_msgs tf2 tf2_ros image_transport camera_info_manager)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)