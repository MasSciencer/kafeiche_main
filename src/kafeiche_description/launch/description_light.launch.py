import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command  # Импортируем Command
from launch_ros.parameter_descriptions import ParameterValue  # Импортируем ParameterValue

#from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Найдем путь к вашему пакету
    package_dir = get_package_share_directory('kafeiche_description')
    urdf_file = os.path.join(package_dir, 'urdf', 'kafeiche_base.xacro')

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
            }]
        ),

        Node(
            package='kafeiche_drivers',
            executable='encoders',
            output='screen',
            parameters=[{
            }]
        ),

        Node(
            package='kafeiche_drivers',
            executable='motor',
            output='screen',
            }]
        ),

        Node(
            package='kafeiche_drivers',
            executable='diffriver',
            output='screen',
            }]
        ),
    )
])

