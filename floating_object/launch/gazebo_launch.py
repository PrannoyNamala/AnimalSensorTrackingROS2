from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_ros_path = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    world_path = os.path.join(get_package_share_directory('floating_object'), 'worlds', 'empty.world')
    topic_name_pos_object = "/model/floating_object/pose"
    log_level = "info"
    use_sim_time = True

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(gazebo_ros_path, 'gzserver.launch.py')]),
            launch_arguments={'world': world_path}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(gazebo_ros_path, 'gzclient.launch.py')]),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="log",
            arguments=[
                topic_name_pos_object + "@geometry_msgs/msg/PoseStamped@gz.msgs.Pose",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[(topic_name_pos_object, "/object_pose")],
        ),
    ])

