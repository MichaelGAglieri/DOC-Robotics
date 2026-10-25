# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 Michael Giuseppe Aglieri
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    remap = [('/user/voice_text', '/laboratorio/testo'),
             ('/robot/speech', '/laboratorio/risposta'),
             ('/robot/tts_status', '/laboratorio/tts_status'),
             ('/llm_prompt', '/laboratorio/llm_prompt')]
    args = [DeclareLaunchArgument('con_voce', default_value='false'),
            DeclareLaunchArgument('con_llm', default_value='false'),
            DeclareLaunchArgument('command_port', default_value='9999'),
            DeclareLaunchArgument('feedback_port', default_value='9998')]
    core = Node(package='doc_lab', executable='laboratorio_node', output='screen',
                parameters=[{'con_llm': ParameterValue(LaunchConfiguration('con_llm'), value_type=bool),
                             'command_port': ParameterValue(LaunchConfiguration('command_port'), value_type=int),
                             'feedback_port': ParameterValue(LaunchConfiguration('feedback_port'), value_type=int)}])
    voice = [Node(package='doc_lab', executable=executable, namespace='laboratorio', output='screen',
                  remappings=remap, condition=IfCondition(LaunchConfiguration('con_voce')))
             for executable in ('stt_node', 'tts_node')]
    llm = Node(package='doc_lab', executable='llm_brain_node', namespace='laboratorio', output='screen',
               remappings=remap, condition=IfCondition(LaunchConfiguration('con_llm')))
    return LaunchDescription(args + [core] + voice + [llm])
