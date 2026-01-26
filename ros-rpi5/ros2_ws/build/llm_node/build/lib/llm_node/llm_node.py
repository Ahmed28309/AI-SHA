#!/usr/bin/env python3
"""
ROS2 LLM Node using Llama 3.2 1B
Subscribes to /speech_rec and publishes LLM responses to /speech/text
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from llama_cpp import Llama


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        # Parameters - Optimized for fast inference on RPI5
        self.declare_parameter('model_path', os.path.expanduser('~/llm_models/Llama-3.2-1B-Instruct-Q4_K_M.gguf'))
        self.declare_parameter('n_ctx', 512)  # Reduced for faster inference
        self.declare_parameter('n_threads', 4)
        self.declare_parameter('temperature', 0.7)  # Slightly lower for more focused responses
        self.declare_parameter('max_tokens', 60)  # Reduced for faster generation
        self.declare_parameter('top_p', 0.9)  # Nucleus sampling for speed
        self.declare_parameter('top_k', 40)  # Limited sampling for speed
        self.declare_parameter('system_prompt_file', os.path.expanduser('~/ros2_ws/src/llm_node/system_prompt.txt'))
        self.declare_parameter('system_prompt', 'You are a helpful AI assistant. Keep responses brief and conversational.')

        model_path = self.get_parameter('model_path').value
        n_ctx = self.get_parameter('n_ctx').value
        n_threads = self.get_parameter('n_threads').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.top_p = self.get_parameter('top_p').value
        self.top_k = self.get_parameter('top_k').value
        system_prompt_file = self.get_parameter('system_prompt_file').value

        # Load system prompt from file if it exists
        if os.path.isfile(system_prompt_file):
            try:
                with open(system_prompt_file, 'r') as f:
                    self.system_prompt = f.read().strip()
                self.get_logger().info('AI-SHARJAH Brain: Personality loaded ✓')
            except Exception as e:
                self.system_prompt = self.get_parameter('system_prompt').value
        else:
            self.system_prompt = self.get_parameter('system_prompt').value

        # Verify model exists
        if not os.path.isfile(model_path):
            self.get_logger().error('AI-SHARJAH Brain: Model not found')
            raise FileNotFoundError(f'Model not found at {model_path}')

        # Load LLM
        self.get_logger().info('AI-SHARJAH Brain: Loading intelligence...')
        try:
            self.llm = Llama(
                model_path=model_path,
                n_ctx=n_ctx,
                n_threads=n_threads,
                n_gpu_layers=0,  # CPU only for Pi 5
                verbose=False
            )
            self.get_logger().info('AI-SHARJAH Brain: Ready to think ✓')
        except Exception as e:
            self.get_logger().error('AI-SHARJAH Brain: Failed to load')
            raise

        # Conversation history
        self.conversation_history = []
        self.max_history = 3  # Keep last 3 exchanges for speed

        # Publisher and Subscriber
        self.publisher = self.create_publisher(String, '/speech/text', 10)
        self.subscription = self.create_subscription(
            String,
            '/speech_rec',
            self.speech_callback,
            10
        )

    def speech_callback(self, msg):
        """Process incoming speech and generate LLM response"""
        user_input = msg.data.strip()

        if not user_input:
            return

        self.get_logger().info(f'AI-SHARJAH Brain: Thinking...')

        try:
            # Generate response
            response = self.generate_response(user_input)

            if response:
                self.get_logger().info(f'AI-SHARJAH Brain: Response → "{response}"')

                # Publish to TTS
                response_msg = String()
                response_msg.data = response
                self.publisher.publish(response_msg)

                # Update conversation history
                self.conversation_history.append({
                    'role': 'user',
                    'content': user_input
                })
                self.conversation_history.append({
                    'role': 'assistant',
                    'content': response
                })

                # Trim history if too long
                if len(self.conversation_history) > self.max_history * 2:
                    self.conversation_history = self.conversation_history[-(self.max_history * 2):]

        except Exception as e:
            pass

    def generate_response(self, user_input):
        """Generate LLM response using Llama 3.2"""
        # Build messages with conversation history
        messages = [
            {
                'role': 'system',
                'content': self.system_prompt
            }
        ]

        # Add conversation history
        messages.extend(self.conversation_history)

        # Add current user input
        messages.append({
            'role': 'user',
            'content': user_input
        })

        # Generate response - optimized for speed
        response = self.llm.create_chat_completion(
            messages=messages,
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            top_p=self.top_p,
            top_k=self.top_k,
            repeat_penalty=1.1,
            stop=['<|eot_id|>', '\n\n', 'User:', 'Question:'],
        )

        # Extract response text
        if response and 'choices' in response and len(response['choices']) > 0:
            response_text = response['choices'][0]['message']['content'].strip()
            return response_text

        return None


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LLMNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
