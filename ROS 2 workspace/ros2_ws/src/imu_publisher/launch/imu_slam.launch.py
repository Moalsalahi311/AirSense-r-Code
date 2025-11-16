from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU publisher node
        Node(
            package="imu_publisher",
            executable="imu_node",
            name="imu_node",
            output="screen"
        ),

        # Madgwick filter
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="madgwick_filter",
            output="screen",
            remappings=[
                ("/imu/data_raw", "/imu/data_raw"),
                ("/imu/data", "/imu/data")
            ]
        ),

        # EKF (robot_localization)
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=["config/ekf.yaml"]
        ),

        # slam_toolbox (online async)
        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[{"use_sim_time": False}],
        ),

        # RViz for visualization
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        ),
    ])

