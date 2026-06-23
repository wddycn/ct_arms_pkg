cmake_minimum_required(VERSION 3.8)
project(eyou_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(xacro REQUIRED)

# 头文件路径，仅保留顶层include，内层目录代码自动拼接
include_directories(
  include
)

# 编译硬件插件动态库
add_library(eyou_system_plugin SHARED
  src/eyou_system.cpp
)
target_include_directories(eyou_system_plugin PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(eyou_system_plugin
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# 【已完全注释，无驱动库链接，不会报cannot find -leyou_driver】
# target_link_directories(eyou_system_plugin PRIVATE /path/to/your/driver/lib)
# target_link_libraries(eyou_system_plugin eyou_driver)

# 注册插件描述文件
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# 安装规则
install(TARGETS
  eyou_system_plugin
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME}/)

# 导出包依赖
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp hardware_interface pluginlib rclcpp_lifecycle
  ros2_control ros2_controllers xacro
)
ament_export_include_directories(include)

ament_package()
