from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    gui = LaunchConfiguration('gui', default='true')
    gui_width_min = LaunchConfiguration('gui_width_min_mm', default='33.0')
    gui_width_max = LaunchConfiguration('gui_width_max_mm', default='71.0')
    model = PathJoinSubstitution([
        FindPackageShare('onrobot_2fg7_description'),
        'urdf',
        'onrobot_2fg7.urdf.xacro',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('onrobot_2fg7_description'),
        'rviz',
        'urdf.rviz',
    ])
    robot_description = Command(['xacro ', model])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Open joint_state_publisher_gui and bridge its commands to the real gripper',
        ),
        DeclareLaunchArgument(
            'gui_width_min_mm',
            default_value='33.0',
            description='Minimum real 2FG7 width mapped from the GUI slider',
        ),
        DeclareLaunchArgument(
            'gui_width_max_mm',
            default_value='71.0',
            description='Maximum real 2FG7 width mapped from the GUI slider',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
            remappings=[('/joint_states', '/onrobot_2fg7/gui_joint_states')],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
            remappings=[('/joint_states', '/onrobot_2fg7/joint_states')],
        ),
        Node(
            package='onrobot_2fg7_tutorials',
            executable='onrobot_2fg7_service',
            name='onrobot_2fg7_service',
            output='screen',
        ),
        Node(
            package='onrobot_2fg7_tutorials',
            executable='onrobot_2fg7_gui_bridge',
            name='onrobot_2fg7_gui_bridge',
            output='screen',
            condition=IfCondition(gui),
            parameters=[{
                'gui_joint_states_topic': '/onrobot_2fg7/gui_joint_states',
                'move_service': '/onrobot_2fg7/move',
                'width_min_mm': gui_width_min,
                'width_max_mm': gui_width_max,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
