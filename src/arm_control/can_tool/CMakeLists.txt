cmake_minimum_required(VERSION 3.8)
project(can_tool)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 添加依赖包

# 添加头文件搜索路径
include_directories(include)
# 添加可执行文件
add_executable(can_tool src/can_tool.cpp)
# 添加依赖即链接库

# 添加安装规则
install(TARGETS
can_tool
DESTINATION lib/${PROJECT_NAME})

# 分享头文件的路径
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION include/${PROJECT_NAME})

# 导出头文件路径
ament_export_include_directories(include)



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