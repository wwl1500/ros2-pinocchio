cmake_minimum_required(VERSION 3.8)
project(pinocchio_ros2_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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
add_executable(fluid_dynamics_cpp_node
  src/fluid_dynamics_cpp/fluid_dynamics_cpp_node.cpp
)

# ROS2 依赖
ament_target_dependencies(
  pinocchio_node
  rclcpp
  std_msgs
  sensor_msgs
)

ament_target_dependencies(
  fluid_dynamics_cpp_node
  rclcpp
  std_msgs
  sensor_msgs
)

# 设置包含目录
target_include_directories(pinocchio_node PUBLIC
  ${PINOCCHIO_INCLUDE_DIRS}
  /usr/include/eigen3
)

target_include_directories(fluid_dynamics_cpp_node PUBLIC
  ${PINOCCHIO_INCLUDE_DIRS}
  /usr/include/eigen3
)

# 链接库目录
if(PINOCCHIO_LIBRARY_DIRS)
  target_link_directories(pinocchio_node PUBLIC ${PINOCCHIO_LIBRARY_DIRS})
  target_link_directories(fluid_dynamics_cpp_node PUBLIC ${PINOCCHIO_LIBRARY_DIRS})
endif()

# 链接库
target_link_libraries(pinocchio_node
  ${PINOCCHIO_LIBRARIES}
)

target_link_libraries(fluid_dynamics_cpp_node
  ${PINOCCHIO_LIBRARIES}
)

# 添加编译选项
if(PINOCCHIO_CFLAGS_OTHER)
  target_compile_options(pinocchio_node PUBLIC ${PINOCCHIO_CFLAGS_OTHER})
  target_compile_options(fluid_dynamics_cpp_node PUBLIC ${PINOCCHIO_CFLAGS_OTHER})
endif()

# 安装可执行文件
install(TARGETS
  pinocchio_node
  fluid_dynamics_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 脚本（如果使用）
install(PROGRAMS
  scripts/pinocchio_python_node.py
  scripts/fluid_dynamics_node.py
  scripts/joint_state_sine_pub.py
  scripts/fluid_dynamics_verifier.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装示例模型
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# 安装参数配置
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

