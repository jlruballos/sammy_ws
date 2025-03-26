import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    package_name = 'sammy_sim'
    json_file_path = os.path.join(get_package_share_directory(package_name), 'motion', 'wave.json')
    
    # Path to Xacro file
    xacro_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'sammy.xacro')

    # Convert Xacro to URDF at runtime
    robot_description = {'robot_description': Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file_path
    ])}

    return launch.LaunchDescription([
        # Joint State Publisher GUI
        launch_ros.actions.Node(
            package='joint_state_publisher_gui',  # GUI version
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        # Robot State Publisher
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        # RViz2
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'urdf.rviz')]
        ),

        # # Launch JSON Joint Publisher
        # launch_ros.actions.Node(
        #     package='sammy_sim',
        #     executable='json_joint_publisher.py',
        #     name='json_joint_publisher',
        #     output='screen',
        #     parameters=[{'json_file': json_file_path}]
        # ),
    ])
