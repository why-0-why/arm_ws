cmake_minimum_required(VERSION 3.8)
project(angle_read)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 添加依赖包
find_package(rclcpp REQUIRED)
find_package(can_tool REQUIRED)# can消息收发
find_package(servo_motor_interface REQUIRED)# 伺服电机接口
find_package(servo_motor_control REQUIRED)# 控制电机接口

include_directories(include)
# 添加可执行文件
add_executable(angle_read_node src/angle_read_node.cpp)

# 添加依赖即链接库
ament_target_dependencies(angle_read_node
rclcpp
can_tool
servo_motor_interface
servo_motor_control
)

# 添加安装规则
install(TARGETS
angle_read_node
DESTINATION lib/${PROJECT_NAME})
# 设置启动文件的路径
# install(DIRECTORY launch
# DESTINATION share/${PROJECT_NAME})

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
