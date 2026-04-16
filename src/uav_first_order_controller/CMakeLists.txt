cmake_minimum_required(VERSION 3.8)
project(uav_first_order_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 编译6个无人机控制器（uav1~uav6）
set(controller_files
  uav1_controller
  uav2_controller
  uav3_controller
  uav4_controller
  uav5_controller
  uav6_controller
)

foreach(controller ${controller_files})
  add_executable(${controller} src/${controller}.cpp)
  ament_target_dependencies(${controller}
    rclcpp
    px4_msgs
    geometry_msgs
  )
  # 安装可执行文件到lib目录
  install(TARGETS ${controller}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
