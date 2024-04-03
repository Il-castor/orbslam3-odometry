from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()

    
    config_node = os.path.join(
        
        get_package_share_directory('orbslam3_odometry'),
        'config',
        'orbslam3_odometry.yaml'
    )
    # config_intrinsic = os.path.join(
    #     get_package_share_directory('driver_camera'),
    #     'config',
    #     'intrinsic_params.yaml'
    #     )

    node=Node(
            package='orbslam3_odometry',
            # namespace='driver_camera',
            name='orbslam3_odometry_node',
            executable='orbslam3_odometry',
            output = 'screen',
            #parameters=[config_node, config_intrinsic]
            parameters=[config_node]
        )

    ld.add_action(node)
    return ld