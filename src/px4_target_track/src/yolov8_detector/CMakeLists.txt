cmake_minimum_required(VERSION 3.8)
project(yolov8_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)

# 查找NumPy头文件
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
  OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# YOLOv8 detector library
add_library(yolo_detector
  src/YOLOv8Detector.cpp
)

target_include_directories(yolo_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Python3_INCLUDE_DIRS}
  ${NUMPY_INCLUDE_DIR}
)

target_link_libraries(yolo_detector
  ${Python3_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(yolo_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)

# YOLOv8 detector node executable
add_executable(yolov8_detector_node
  src/yolov8_detector_node.cpp
)

target_include_directories(yolov8_detector_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Python3_INCLUDE_DIRS}
  ${NUMPY_INCLUDE_DIR}
)

target_link_libraries(yolov8_detector_node
  yolo_detector
  ${Python3_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(yolov8_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
)

install(TARGETS yolov8_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# 安装库文件
install(TARGETS yolo_detector
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  OpenCV
  Python3
)

ament_package()
