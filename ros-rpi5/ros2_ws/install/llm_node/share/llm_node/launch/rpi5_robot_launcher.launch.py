#!/usr/bin/env python3
"""
RPI5 Robot Complete System Launch File
Launches all nodes for AI-SHARJAH (ISC Sharjah Educational Robot):
- Speech Recognition (STT) Node
- LLM Processing Node
- Text-to-Speech (TTS) Node
- BNO055 IMU Node

Usage:
    ros2 launch llm_node rpi5_robot_launcher.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Declare launch arguments for easy customization
    declare_whisper_model = DeclareLaunchArgument(
        'whisper_model',
        default_value='base.en',
        description='Whisper model size (tiny.en, base.en, small.en)'
    )

    declare_audio_device = DeclareLaunchArgument(
        'audio_device',
        default_value='plughw:3,0',
        description='Audio input device for microphone'
    )

    declare_llm_max_tokens = DeclareLaunchArgument(
        'llm_max_tokens',
        default_value='60',
        description='Maximum tokens for LLM response (reduced for speed)'
    )

    declare_system_prompt_file = DeclareLaunchArgument(
        'system_prompt_file',
        default_value=os.path.expanduser('~/ros2_ws/src/llm_node/system_prompt.txt'),
        description='Path to system prompt file'
    )

    # Speech Recognition Node (STT)
    stt_node = Node(
        package='speech_recognition',
        executable='stt_node',
        name='stt_node',
        output='screen',
        parameters=[{
            'model_size': LaunchConfiguration('whisper_model'),
            'device': 'cpu',
            'compute_type': 'int8',
            'audio_device': LaunchConfiguration('audio_device'),
            'sample_rate': 16000,
            'channels': 6,
            'chunk_duration': 2,
            'energy_threshold': 100.0
        }],
        emulate_tty=True
    )

    # LLM Processing Node
    llm_node = Node(
        package='llm_node',
        executable='llm_node',
        name='llm_node',
        output='screen',
        parameters=[{
            'model_path': os.path.expanduser('~/llm_models/Llama-3.2-1B-Instruct-Q4_K_M.gguf'),
            'n_ctx': 512,
            'n_threads': 4,
            'temperature': 0.7,
            'max_tokens': LaunchConfiguration('llm_max_tokens'),
            'top_p': 0.9,
            'top_k': 40,
            'system_prompt_file': LaunchConfiguration('system_prompt_file'),
            'system_prompt': 'You are a helpful AI assistant.'
        }],
        emulate_tty=True
    )

    # Text-to-Speech Node (TTS)
    tts_node = Node(
        package='tts_speaker',
        executable='tts_speaker_node',
        name='tts_speaker_node',
        output='screen',
        emulate_tty=True
    )

    # BNO055 IMU Node
    imu_node = Node(
        package='bno055_imu',
        executable='bno055_node.py',
        name='bno055_imu_node',
        output='screen',
        emulate_tty=True
    )

    # Startup message
    startup_msg = LogInfo(
        msg=[
            '\n',
            '='*70, '\n',
            '  AI-SHARJAH - ISC Sharjah Educational Robot Starting...\n',
            '='*70, '\n',
            '  Nodes:\n',
            '    - Speech Recognition (STT): /speech_rec\n',
            '    - LLM Processing: /speech/text\n',
            '    - Text-to-Speech (TTS): Audio Output\n',
            '    - BNO055 IMU: /imu/data\n',
            '='*70, '\n'
        ]
    )

    return LaunchDescription([
        # Launch arguments
        declare_whisper_model,
        declare_audio_device,
        declare_llm_max_tokens,
        declare_system_prompt_file,

        # Startup message
        startup_msg,

        # All nodes
        stt_node,
        llm_node,
        tts_node,
        imu_node
    ])
