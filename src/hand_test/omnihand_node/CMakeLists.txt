cmake_minimum_required(VERSION 3.8)
project(omnihand_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(omnihand_node_msgs REQUIRED)
# find_package(omni_hand_driver REQUIRED)

# #############桥接sdk############
# SDK 预构建安装路径
set(OMNIHAND_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Omnihand-2025-SDK/build/install")

# 创建 IMPORTED 库目标
add_library(omnihand2025 SHARED IMPORTED)
set_target_properties(omnihand2025 PROPERTIES
  IMPORTED_LOCATION "${OMNIHAND_SDK_DIR}/lib/libomnihand2025.so"
  INTERFACE_INCLUDE_DIRECTORIES "${OMNIHAND_SDK_DIR}/include"
)

# 安装 SDK 头文件
install(DIRECTORY ${OMNIHAND_SDK_DIR}/include/
  DESTINATION include/omnihand_node)

# 安装 SDK 共享库
install(FILES
  ${OMNIHAND_SDK_DIR}/lib/libomnihand2025.so
  ${OMNIHAND_SDK_DIR}/lib/libusbcanfd.so
  DESTINATION lib)

# 安装 libusbcanfd 的版本号符号链接
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
  libusbcanfd.so \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/libusbcanfd.so.1.0.10)")


# 导出 CMake 配置，让下游包通过 find_package(omnihand_node) 找到库
ament_export_include_directories(include/omnihand_node)
ament_export_libraries(omnihand2025 usbcanfd)

####################################

add_executable(hand_node src/main.cpp src/hand_node.cpp)


# SDK 需要 C++20
target_compile_features(hand_node PUBLIC c_std_99 cxx_std_20)

# 包含本包头文件
target_include_directories(hand_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(
  hand_node
    "rclcpp"
    "std_msgs"
    "rosidl_typesupport_cpp"
    "rosidl_typesupport_fastrtps_cpp"
    "rosidl_default_generators"
    "omnihand_node_msgs"
)


# 链接 SDK 库（同时传递 INTERFACE_INCLUDE_DIRECTORIES）
target_link_libraries(hand_node omnihand2025)

install(TARGETS hand_node
  DESTINATION lib/${PROJECT_NAME})


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
