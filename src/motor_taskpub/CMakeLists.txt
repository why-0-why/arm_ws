cmake_minimum_required(VERSION 3.8)
project(motor_taskpub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# 1.添加依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(servo_motor_interface REQUIRED)# 伺服电机接口
find_package(motor_status_display REQUIRED)# 电机状态显示
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# 2.创建可执行文件
add_executable(motor_taskpub_node
src/motor_taskpub.cpp)# （可执行文件名），（源文件）
# 3.设置依赖关系
ament_target_dependencies(motor_taskpub_node 
rclcpp
servo_motor_interface
)# (可执行文件名)，（依赖的包）
# 4.设置安装路径
install(TARGETS 
motor_taskpub_node  
DESTINATION lib/${PROJECT_NAME})# (可执行文件名)，（安装路径）
# 设置启动文件的路径
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

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
