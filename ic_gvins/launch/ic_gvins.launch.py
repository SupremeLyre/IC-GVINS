from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("ic_gvins")
    configfile = LaunchConfiguration("configfile")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    rviz_config = PathJoinSubstitution([package_share, "config", "visualization.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "configfile",
                default_value=PathJoinSubstitution([package_share, "config", "gvins.yaml"]),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="ic_gvins",
                executable="ic_gvins_ros",
                name="ic_gvins_node",
                output="screen",
                parameters=[
                    {
                        "configfile": configfile,
                        "use_sim_time": use_sim_time,
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
                condition=IfCondition(LaunchConfiguration("rviz")),
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["-d", rviz_config],
            ),
        ]
    )
