from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # ✅ Static transform: World to Map (if world is required)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "map"]
        ),

        # ✅ Static transform for Base to Odom
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
        ),

        # ✅ Static transform for Base to LiDAR
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "1.0", "0", "0", "0", "base_link", "lidar_link"]
        ),

        # ✅ Static transform for IMU (if needed)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "imu_link"]
        ),

        # ✅ Static transform for GPS (if needed)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0.2", "0", "0", "0", "base_link", "navsat_link"]
        ),

        # ✅ LiDAR IMU TF Node (Ensure this executable exists in package)
        Node(
            package="unitree_lidar_mapping",
            executable="lidar_imu_tf_node",
            name="lidar_imu_tf_node",
            output="screen",
            parameters=[{"use_sim_time": False}]
        ),

        # ✅ LiDAR Mapping Node (Ensure executable exists)
        Node(
            package="unitree_lidar_mapping",
            executable="lidar_mapping_node",
            name="lidar_mapping_node",
            output="screen",
            parameters=[{"use_sim_time": False}]
        ),

        # ✅ RViz2 with Correct Path Handling
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", os.path.join(
                os.getenv("HOME"), "ros2_foxy/src/unitree_lidar_mapping/config/mapping_config.rviz"
            )]
        )
    ])

