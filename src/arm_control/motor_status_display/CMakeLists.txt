cmake_minimum_required(VERSION 3.8)
project(motor_status_display)

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
find_package(Qt5 REQUIRED COMPONENTS Widgets)# 依赖Qt5库

find_package(can_tool REQUIRED)# can接口
find_package(servo_motor_interface REQUIRED)# 伺服电机接口
find_package(servo_motor_control REQUIRED)# 伺服电机接口
find_package(angle_read REQUIRED)# 控制器读取角度 

# 添加头文件搜索路径
include_directories(include)
# 添加可执行文件
add_executable(motor_status_display_node
src/motor_status_display.cpp)# （可执行文件名），（源文件）
# 添加依赖即链接库
ament_target_dependencies(motor_status_display_node 
rclcpp
servo_motor_interface
can_tool
servo_motor_control
)# (可执行文件名)，（依赖的包）
# 链接Qt5库
target_link_libraries(motor_status_display_node 
  Qt5::Widgets
)

# 添加可执行文件路径
install(TARGETS 
motor_status_display_node  
DESTINATION lib/${PROJECT_NAME})# (可执行文件名)，（安装路径）
# 添加启动文件的路径
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

# 分享头文件的路径
# 导出头文件路径

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
