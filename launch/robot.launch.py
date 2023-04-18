from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='segwayrmp',
#             #namespace='turtlesim1',
#             executable='SmartCar',
#             name='SmartCar',
#             output='screen'
#         ),
#         Node(
#             package='joy',
#             #namespace='turtlesim2',
#             executable='joy_node',
#             name='joy_teleop',
#             output='screen'

#         ),
#         Node(
#             package='rmp220_teleop',
#             executable='rmp220_teleop',
#             name='rmp220_teleop',
#             # remappings=[
#             #     ('/input/pose', '/turtlesim1/turtle1/pose'),
#             #     ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
#             # ]
#             output='screen'

#         )
#     ])

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    #joy_params = os.path.join(get_package_share_directory('rmp220_teleop'))

    joy_node = Node(
            package='joy',
            executable='joy_node',
            #parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='rmp220_teleop',
            executable='rmp220_teleop',
            name='rmp220_teleop',
            #parameters=[joy_params, {'use_sim_time': use_sim_time}],
            #remappings=[('/cmd_vel','/cmd_vel_joy')]
         )
    
    control_node = Node(
            package='segwayrmp',
            executable='SmartCar',
            name='SmartCar',
            #parameters=[joy_params, {'use_sim_time': use_sim_time}],
            #remappings=[('/cmd_vel','/cmd_vel_joy')]
         )

    # twist_stamper = Node(
    #         package='twist_stamper',
    #         executable='twist_stamper',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
    #                     ('/cmd_vel_out','/diff_cont/cmd_vel')]
    #      )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
        control_node,
        # twist_stamper       
    ])