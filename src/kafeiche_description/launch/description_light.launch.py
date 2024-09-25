from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kafeiche_description"), "urdf", "kafeiche_base.xacro"]
            ),
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("kafeiche_drivers"),
            "config",
            "kafeiche_controllers.yaml",
        ]
    )
    
    #added
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    #added
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    #added
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    #added throughout delay below
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    #added
    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    #added
    hardware_encoder = Node(
        package='kafeiche_drivers',
        executable='encoder',
        output='screen',
        parameters=[{
        }]
    )

    #added
    hardware_motor = Node(
        package='kafeiche_drivers',
        executable='motor',
        output='screen',
        parameters=[{
        }]
    )

    #summary
    nodes = [
        #control_node, //Needed test
        #robot_state_pub_node, //Needed test
        #joint_state_broadcaster_spawner, //Needed test
        hardware_encoder,
        hardware_motor,
        #delay_robot_controller_spawner_after_joint_state_broadcaster_spawner, //Needed test
    ]

    return LaunchDescription(nodes)
