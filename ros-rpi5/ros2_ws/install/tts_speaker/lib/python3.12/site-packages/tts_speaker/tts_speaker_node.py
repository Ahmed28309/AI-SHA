#!/usr/bin/env python3
"""
ROS2 TTS Speaker Node
Subscribes to /speech/text and speaks the text using Piper TTS
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import subprocess
import os


class TTSSpeakerNode(Node):
    def __init__(self):
        super().__init__('tts_speaker_node')

        # Piper configuration
        self.piper_bin = os.path.expanduser('~/.local/bin/piper')
        self.piper_model = os.path.expanduser('~/piper_models/en_US-lessac-medium.onnx')
        self.audio_device = 'plughw:0,0'

        # Verify piper and model exist
        if not os.path.isfile(self.piper_bin):
            self.get_logger().error('AI-SHARJAH Voice: Piper not found')
            raise FileNotFoundError(f'Piper not found at {self.piper_bin}')

        if not os.path.isfile(self.piper_model):
            self.get_logger().error('AI-SHARJAH Voice: Model not found')
            raise FileNotFoundError(f'Piper model not found at {self.piper_model}')

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            '/tts_text',
            self.speech_callback,
            10
        )

        # Create publisher for speaking state (to prevent mic feedback)
        self.speaking_publisher = self.create_publisher(Bool, '/robot/speaking', 10)

        self.get_logger().info('AI-SHARJAH Voice: Ready to speak ✓')

    def speech_callback(self, msg):
        """Callback function to handle incoming text messages"""
        text = msg.data.strip()

        if not text:
            return

        self.get_logger().info(f'AI-SHARJAH Voice: Speaking → "{text}"')

        # Publish speaking state = True (mutes microphone)
        speaking_msg = Bool()
        speaking_msg.data = True
        self.speaking_publisher.publish(speaking_msg)

        try:
            # Generate speech with Piper and pipe to aplay
            piper_process = subprocess.Popen(
                [
                    self.piper_bin,
                    '--model', self.piper_model,
                    '--output-raw'
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            aplay_process = subprocess.Popen(
                [
                    'aplay',
                    '-D', self.audio_device,
                    '-r', '22050',
                    '-f', 'S16_LE',
                    '-t', 'raw',
                    '-'
                ],
                stdin=piper_process.stdout,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            # Send text to piper
            piper_process.stdin.write(text.encode('utf-8'))
            piper_process.stdin.close()

            # Wait for completion
            aplay_process.wait()
            piper_process.wait()

        except Exception as e:
            self.get_logger().error(f'AI-SHARJAH Voice: TTS Error - {str(e)}')
        finally:
            # Publish speaking state = False (unmutes microphone)
            speaking_msg = Bool()
            speaking_msg.data = False
            self.speaking_publisher.publish(speaking_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TTSSpeakerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
