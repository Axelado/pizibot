
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the dataset_saver node for collecting RGB images and binary masks."""

    package_name = 'pizibot_gz'
    
    label = LaunchConfiguration('label')
    topic_rgb = LaunchConfiguration('topic_rgb')
    topic_sem = LaunchConfiguration('topic_sem')
    dataset_path = LaunchConfiguration('dataset_path')
    
    # Arguments
    label_arg = DeclareLaunchArgument(
        'label',
        default_value="1",
        description='Semantic class label (pixels with this label -> black in mask)'
    )

    topic_rgb_arg = DeclareLaunchArgument(
        'topic_rgb',
        default_value="/camera/image_raw",
        description='ROS2 topic for the RGB image'
    )
    
    topic_sem_arg = DeclareLaunchArgument(
        'topic_sem',
        default_value="/camera/semantic/labels_map",
        description='ROS2 topic for the semantic image (labels)'
    )
    
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value="dataset",
        description='Output folder path (will create images/ and masks/ subfolders)'
    )
    
    dataset_saver_node = Node(
        package=package_name,
        executable='dataset_saver',
        parameters=[{
            'label' : label,
            'topic_rgb' : topic_rgb,
            'topic_sem' : topic_sem,
            'dataset_path' : dataset_path,
        }]
    )
    
    ld = LaunchDescription()
    
    ld.add_action(label_arg)
    ld.add_action(topic_rgb_arg)
    ld.add_action(topic_sem_arg)
    ld.add_action(dataset_path_arg)
    ld.add_action(dataset_saver_node)

    return ld
