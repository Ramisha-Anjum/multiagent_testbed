from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

class LaunchDescriptionGenerator():
    def __init__(self):
        self.vicon_topic = LaunchConfiguration('vicon_topic')
        self.frame_id = LaunchConfiguration('frame_id')
        
    def render_launch_config(self, context: LaunchContext, launch_description: LaunchDescription):
        vicon_topic = context.perform_substitution(self.vicon_topic)
        frame_id = context.perform_substitution(self.frame_id)
        
        # Hardcoded calibration file path
        calibration_file = os.path.join(get_package_share_directory('vicon2pose'), 'config', 'calibration_info.yaml')

        # Launch the vicon2pose node with the provided arguments
        vicon2pose = Node(
            package='vicon2pose',
            executable='vicon2pose_node.py',
            output='screen',
            arguments=[
                "--vicon_topic", vicon_topic,
                "--frame_id", frame_id,
                "--calibration_file", calibration_file  # Hardcoded calibration file path
            ]
        )

        launch_description.add_action(vicon2pose)

    def generate(self):
        launch_description = LaunchDescription()
        
        # Declare the launch arguments
        vicon_topic_arg = DeclareLaunchArgument('vicon_topic', default_value="/vicon/limo01/limo01")
        frame_id_arg = DeclareLaunchArgument('frame_id', default_value='vicon')

        opaque_function = OpaqueFunction(function=self.render_launch_config, args=[launch_description])

        # Add actions to the launch description
        launch_description.add_action(vicon_topic_arg)
        launch_description.add_action(frame_id_arg)
        launch_description.add_action(opaque_function)

        return launch_description

def generate_launch_description():
    launch_description_generator = LaunchDescriptionGenerator()
    return launch_description_generator.generate()
