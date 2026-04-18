cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# =======================================

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc)
find_package(ament_cmake_auto REQUIRED)
find_package(camera_info_manager REQUIRED)
ament_auto_find_build_dependencies()

# =======================================

# # 声明消息文件
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/SensorData.msg"
#   "msg/SensorDataArray.msg"
#   DEPENDENCIES std_msgs
# )

# =======================================

include_directories(
  include
  /opt/MVS/include # Install Hik MVS if this directory is missing
  ${OpenCV_INCLUDE_DIRS}
)

add_library(hik_camera_component SHARED src/hik_camera_node.cpp)

# 链接Hik SDK
target_link_directories(hik_camera_component PRIVATE /opt/MVS/lib/64)

target_link_libraries(hik_camera_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  sensor_msgs::sensor_msgs_library
  image_transport::image_transport
  camera_info_manager::camera_info_manager
  ${OpenCV_LIBRARIES}
  
  # 海康SDK
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
)
# =======================================

# rosidl_target_interfaces(hik_camera_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(hik_camera_component "hik_camera::HikCameraNode")
# 安装共享库到lib目录
install(TARGETS
  hik_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装config目录
# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# =======================================

# 导出依赖
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  camera_info_manager
  opencv
)

ament_package()