from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    teleop_keyboard_node = Node(
            package='teleop_twist_keyboard_for_azerty',
            executable='teleop_twist_keyboard_for_azerty',
            remappings=[('/cmd_vel', '/cmd_vel_key')],
            output='screen',
            # Lancement du nœud dans un vrai terminal avec 'gnome-terminal' ou 'xterm'
            # Choisis l'un d'eux selon le terminal installé sur ton système
            prefix='gnome-terminal --'  # ou 'xterm -e' selon ce que tu as installé
         )

    return LaunchDescription([
        teleop_keyboard_node,
    ])
