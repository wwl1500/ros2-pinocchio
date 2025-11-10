cmake_minimum_required(VERSION 3.8)
project(pinocchio_ros2_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 查找 Pinocchio - 直接使用 pkg-config（最可靠的方法）
find_package(PkgConfig REQUIRED)

# 设置 PKG_CONFIG_PATH 确保能找到 Pinocchio
set(ENV{PKG_CONFIG_PATH} "/opt/openrobots/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

# 使用 pkg-config 获取 Pinocchio 配置
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

# 如果 pkg-config 失败，使用备用路径
if(NOT PINOCCHIO_FOUND)
  message(STATUS "使用备用方法配置 Pinocchio")
  set(PINOCCHIO_INCLUDE_DIRS "/opt/openrobots/include")
  set(PINOCCHIO_LIBRARY_DIRS "/opt/openrobots/lib")
  set(PINOCCHIO_LIBRARIES 
    pinocchio_default
    pinocchio_parsers
    boost_filesystem
    boost_system
  )
  set(PINOCCHIO_CFLAGS_OTHER "-DPINOCCHIO_WITH_URDFDOM")
endif()

message(STATUS "Pinocchio include dirs: ${PINOCCHIO_INCLUDE_DIRS}")
message(STATUS "Pinocchio libraries: ${PINOCCHIO_LIBRARIES}")

# 添加可执行文件
add_executable(pinocchio_node src/pinocchio_node.cpp)

# ROS2 依赖
ament_target_dependencies(
  pinocchio_node
  rclcpp
  std_msgs
)

# 设置包含目录
target_include_directories(pinocchio_node PUBLIC
  ${PINOCCHIO_INCLUDE_DIRS}
  /usr/include/eigen3
)

# 链接库目录
if(PINOCCHIO_LIBRARY_DIRS)
  target_link_directories(pinocchio_node PUBLIC ${PINOCCHIO_LIBRARY_DIRS})
endif()

# 链接库
target_link_libraries(pinocchio_node
  ${PINOCCHIO_LIBRARIES}
)

# 添加编译选项
if(PINOCCHIO_CFLAGS_OTHER)
  target_compile_options(pinocchio_node PUBLIC ${PINOCCHIO_CFLAGS_OTHER})
endif()

# 安装可执行文件
install(TARGETS
  pinocchio_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 脚本（如果使用）
install(PROGRAMS
  scripts/pinocchio_python_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

