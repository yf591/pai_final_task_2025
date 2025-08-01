import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # GazeboにTurtleBot3を出現させる
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 新しい自律制御ノードを起動する設定（xtermは不要）
    autonomous_node = Node(
        package='final_project_pkg',
        executable='task_executor',
        name='task_executor',
        output='screen'
    )

    return LaunchDescription([
        start_world,
        autonomous_node
    ])