#!/usr/bin/env python3
"""Launch file for SABIS Robot LLM Node"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'prompt_type',
            default_value='concise',
            description='Prompt type: "full" or "concise"'
        ),

        DeclareLaunchArgument(
            'max_tokens',
            default_value='200',
            description='Maximum tokens in response'
        ),

        # LLM Node
        Node(
            package='llm_node',
            executable='llm_node',
            name='llm_node',
            output='screen',
            parameters=[{
                'model_path': '/home/orin-robot/models/Llama-3.2-3B-Instruct-Q4_K_M.gguf',
                'system_prompt_path': [
                    '/home/orin-robot/robot_ws/src/llm_node/prompts/sabis_robot_',
                    LaunchConfiguration('prompt_type'),
                    '.txt'
                ],
                'n_ctx': 2048,
                'n_gpu_layers': -1,  # All layers on GPU
                'temperature': 0.7,
                'max_tokens': LaunchConfiguration('max_tokens'),
            }],
        ),
    ])
