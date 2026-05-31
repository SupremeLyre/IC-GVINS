from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("ic_gvins")
    configfile = LaunchConfiguration("configfile")
    rviz_config = PathJoinSubstitution([package_share, "config", "visualization.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "configfile",
                default_value=PathJoinSubstitution([package_share, "config", "gvins.yaml"]),
            ),
            Node(
                package="ic_gvins",
                executable="ic_gvins_ros",
                name="ic_gvins_node",
                output="screen",
                parameters=[
                    {
                        "imu_topic": "/imu0",
                        "gnss_topic": "/gnss0",
                        "image_topic": "/cam0",
                        "configfile": configfile,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_map_broadcaster",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--qx",
                    "-1",
                    "--qy",
                    "0",
                    "--qz",
                    "0",
                    "--qw",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "world",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="visualisation",
                output="log",
                arguments=["-d", rviz_config],
            ),
        ]
    )
