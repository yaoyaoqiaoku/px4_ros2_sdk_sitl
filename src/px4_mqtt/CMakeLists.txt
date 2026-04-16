cmake_minimum_required(VERSION 3.5)
project(px4_mqtt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# Find MQTT library (paho-mqttcpp)
# Manual search for paho-mqttcpp library
find_library(PAHO_MQTT_CPP_LIB 
	NAMES paho-mqttpp3
	PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
	NO_DEFAULT_PATH
)
# Find the C library that C++ library depends on
find_library(PAHO_MQTT_C_LIB 
	NAMES paho-mqtt3as paho-mqtt3a paho-mqtt3c
	PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
	NO_DEFAULT_PATH
)
find_path(PAHO_MQTT_CPP_INCLUDE_DIR 
	NAMES mqtt/client.h
	PATHS /usr/include /usr/local/include
	NO_DEFAULT_PATH
)

if(PAHO_MQTT_CPP_LIB AND PAHO_MQTT_CPP_INCLUDE_DIR)
	message(STATUS "Found paho-mqttcpp: ${PAHO_MQTT_CPP_LIB}")
	message(STATUS "Found paho-mqttcpp headers: ${PAHO_MQTT_CPP_INCLUDE_DIR}")
	if(PAHO_MQTT_C_LIB)
		message(STATUS "Found paho-mqtt C library: ${PAHO_MQTT_C_LIB}")
	endif()
	set(PAHO_MQTT_CPP_FOUND TRUE)
	set(PAHO_MQTT_CPP_LIBRARIES ${PAHO_MQTT_CPP_LIB})
	if(PAHO_MQTT_C_LIB)
		list(APPEND PAHO_MQTT_CPP_LIBRARIES ${PAHO_MQTT_C_LIB})
	endif()
	set(PAHO_MQTT_CPP_INCLUDE_DIRS ${PAHO_MQTT_CPP_INCLUDE_DIR})
else()
	message(FATAL_ERROR "PahoMqttCpp not found. Please install: sudo apt-get install libpaho-mqttpp-dev")
endif()

# Find JSON library (nlohmann/json)
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
	# Try to find it via pkg-config or use header-only version
	find_path(JSON_INCLUDE_DIR nlohmann/json.hpp
		PATHS
		/usr/include
		/usr/local/include
	)
	if(JSON_INCLUDE_DIR)
		message(STATUS "Found nlohmann/json at ${JSON_INCLUDE_DIR}")
		set(JSON_INCLUDE_DIRS ${JSON_INCLUDE_DIR})
	else()
		message(FATAL_ERROR "nlohmann/json not found. Please install: sudo apt-get install nlohmann-json3-dev")
	endif()
endif()

#################
# Setup targets #
#################

include_directories(include)
if(JSON_INCLUDE_DIRS)
	include_directories(${JSON_INCLUDE_DIRS})
endif()
if(PAHO_MQTT_CPP_INCLUDE_DIRS)
	include_directories(${PAHO_MQTT_CPP_INCLUDE_DIRS})
endif()

# MQTT Estimator executable
add_executable(mqtt_estimator 
	src/mqtt_estimator.cpp
)

ament_target_dependencies(mqtt_estimator 
	rclcpp 
	std_msgs
)

# Link MQTT library
if(PAHO_MQTT_CPP_FOUND)
	target_link_libraries(mqtt_estimator 
		${PAHO_MQTT_CPP_LIBRARIES}
	)
	target_include_directories(mqtt_estimator PRIVATE ${PAHO_MQTT_CPP_INCLUDE_DIRS})
else()
	message(FATAL_ERROR "PahoMqttCpp library not found")
endif()

# MQTT Control executable
add_executable(mqtt_control 
	src/mqtt_control.cpp
)

ament_target_dependencies(mqtt_control 
	rclcpp 
	std_msgs
	px4_msgs
)

# Link MQTT library
if(PAHO_MQTT_CPP_FOUND)
	target_link_libraries(mqtt_control 
		${PAHO_MQTT_CPP_LIBRARIES}
	)
	target_include_directories(mqtt_control PRIVATE ${PAHO_MQTT_CPP_INCLUDE_DIRS})
else()
	message(FATAL_ERROR "PahoMqttCpp library not found")
endif()

	# MQTT Home Control executable (new)
	add_executable(mqtt_home_control 
		src/mqtt_home_control.cpp
	)

	ament_target_dependencies(mqtt_home_control 
		rclcpp 
		std_msgs
		px4_msgs
	)

	# Link MQTT library
	if(PAHO_MQTT_CPP_FOUND)
		target_link_libraries(mqtt_home_control 
			${PAHO_MQTT_CPP_LIBRARIES}
		)
		target_include_directories(mqtt_home_control PRIVATE ${PAHO_MQTT_CPP_INCLUDE_DIRS})
	else()
		message(FATAL_ERROR "PahoMqttCpp library not found")
	endif()

# MQTT Mission executable
add_executable(mqtt_mission 
	src/mqtt_mission.cpp
)

ament_target_dependencies(mqtt_mission 
	rclcpp 
	std_msgs
)

# Link MQTT library
if(PAHO_MQTT_CPP_FOUND)
	target_link_libraries(mqtt_mission 
		${PAHO_MQTT_CPP_LIBRARIES}
	)
	target_include_directories(mqtt_mission PRIVATE ${PAHO_MQTT_CPP_INCLUDE_DIRS})
else()
	message(FATAL_ERROR "PahoMqttCpp library not found")
endif()

	# MQTT Servo Control executable (bridge for servo commands over MQTT -> px4_msgs::msg::ServoCommand)
	add_executable(mqtt_servo_control
		src/mqtt_servo_control.cpp
	)

	ament_target_dependencies(mqtt_servo_control
		rclcpp
		px4_msgs
		std_msgs
	)

	# Link MQTT library
	if(PAHO_MQTT_CPP_FOUND)
		target_link_libraries(mqtt_servo_control
			${PAHO_MQTT_CPP_LIBRARIES}
		)
		target_include_directories(mqtt_servo_control PRIVATE ${PAHO_MQTT_CPP_INCLUDE_DIRS})
	else()
		message(FATAL_ERROR "PahoMqttCpp library not found")
	endif()

install(TARGETS mqtt_estimator mqtt_control mqtt_home_control mqtt_mission mqtt_servo_control
	DESTINATION lib/${PROJECT_NAME}
)

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp std_msgs px4_msgs)

# Install header files
install(DIRECTORY include/ DESTINATION include)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_package()
