cmake_minimum_required(VERSION 3.5)
project(unitree_lidar_mapping)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)  # ✅ Add this line for service support



set(CMAKE_CXX_STANDARD 17)  # ✅ Ensure C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include directories (if you have a header folder)
include_directories(include)

add_executable(lidar_imu_tf_node src/lidar_imu_tf_node.cpp)
ament_target_dependencies(lidar_imu_tf_node rclcpp sensor_msgs tf2_ros)



add_executable(lidar_mapping_node src/lidar_mapping_node.cpp)
ament_target_dependencies(lidar_mapping_node rclcpp sensor_msgs octomap octomap_msgs pcl_conversions pcl_ros tf2_ros std_srvs)

# Install the node
install(TARGETS lidar_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (if applicable)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS lidar_imu_tf_node
  DESTINATION lib/${PROJECT_NAME})
  
  
ament_package()

