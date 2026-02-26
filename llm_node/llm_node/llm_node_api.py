#!/usr/bin/env python3
"""
LLM Node API - Ultra-Fast Language Model using Gemini API
20x faster than local Llama model
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import google.generativeai as genai
import threading
import os

from llm_node.chess_handler import ChessHandler


class LLMNodeAPI(Node):
    def __init__(self):
        super().__init__('llm_node')

        # Declare parameters
        self.declare_parameter('api_key', '')
        self.declare_parameter('model_name', 'gemini-2.5-flash')
        self.declare_parameter('system_prompt_path',
                             '/home/orin-robot/robot_ws/src/llm_node/prompts/sabis_robot_system.txt')
        self.declare_parameter('temperature', 0.4)
        self.declare_parameter('max_tokens', 150)

        # Get parameters
        api_key = self.get_parameter('api_key').value
        model_name = self.get_parameter('model_name').value
        system_prompt_path = self.get_parameter('system_prompt_path').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value

        # Check for API key
        if not api_key:
            api_key = os.environ.get('GEMINI_API_KEY', '')

        if not api_key:
            self.get_logger().error('No API key provided! Set GEMINI_API_KEY env var or pass api_key parameter')
            raise ValueError('Missing Gemini API key')

        # Configure Gemini
        genai.configure(api_key=api_key)

        # Load system prompt
        self.system_prompt = self._load_system_prompt(system_prompt_path)

        # Initialize model
        self.get_logger().info(f'Initializing Gemini model: {model_name}...')
        self.model = genai.GenerativeModel(
            model_name=model_name,
            generation_config=genai.GenerationConfig(
                temperature=self.temperature,
            ),
            system_instruction=self.system_prompt
        )
        self.get_logger().info('✓ Gemini model initialized')

        # Initialize publisher and subscriber
        self.publisher = self.create_publisher(String, '/tts_text', 10)
        self.subscription = self.create_subscription(
            String,
            '/speech/text',
            self.speech_callback,
            10)

        # Chess capability (built-in, no extra nodes needed)
        self.chess = ChessHandler()

        self.get_logger().info('🚀 Gemini LLM API ready! Ultra-fast cloud intelligence')

    def _load_system_prompt(self, prompt_path):
        """Load system prompt from file."""
        try:
            if os.path.exists(prompt_path):
                with open(prompt_path, 'r') as f:
                    prompt = f.read().strip()
                self.get_logger().info(f'Loaded system prompt from {prompt_path} ({len(prompt)} chars)')
                return prompt
            else:
                self.get_logger().warn(f'System prompt file not found: {prompt_path}')
                return "You are a helpful robot assistant. Provide brief, clear responses."
        except Exception as e:
            self.get_logger().error(f'Error loading system prompt: {e}')
            return "You are a helpful robot assistant. Provide brief, clear responses."

    def speech_callback(self, msg):
        """Handle incoming speech text and generate LLM response."""
        input_text = msg.data.strip()

        if not input_text:
            return

        # Check if this is chess-related
        if self.chess.is_chess_intent(input_text):
            self.get_logger().info(f'♟ Chess handling: "{input_text}"')
            threading.Thread(target=self._handle_chess,
                            args=(input_text,), daemon=True).start()
            return

        self.get_logger().warn(f'RECEIVED FROM /speech/text: "{input_text}"')
        self.get_logger().warn(f'   Generating Gemini response...')

        # Generate response in a separate thread to not block the executor
        threading.Thread(target=self._generate_response,
                        args=(input_text,),
                        daemon=True).start()

    def _handle_chess(self, text):
        """Process chess input and publish response."""
        try:
            response = self.chess.handle(text)
            if response:
                self._publish(response)
                self.get_logger().info(f'Chess response: "{response}"')
        except Exception as e:
            self.get_logger().error(f'Chess error: {e}')
            self._publish("Something went wrong with the chess game. Let's start over.")
            self.chess.active = False

    def _generate_response(self, input_text):
        """Generate Gemini response and publish it."""
        try:
            start_time = __import__('time').time()
            self.get_logger().info('Generating Gemini response...')

            # Detect if input contains Arabic
            is_arabic = any(ord(c) >= 0x0600 and ord(c) <= 0x06FF for c in input_text)
            language_instruction = ""

            if is_arabic:
                language_instruction = "\n\nIMPORTANT: The user spoke in Arabic. You MUST respond ENTIRELY in Arabic."
                self.get_logger().info('Arabic detected - instructing LLM to respond in Arabic')
            else:
                language_instruction = "\n\nIMPORTANT: The user spoke in English. You MUST respond ENTIRELY in English."
                self.get_logger().info('English detected - instructing LLM to respond in English')

            # Generate response with language instruction
            full_prompt = input_text + language_instruction
            response = self.model.generate_content(full_prompt)

            # Extract text
            response_text = response.text.strip()

            elapsed = __import__('time').time() - start_time

            if response_text:
                self.get_logger().warn(f'Gemini response ({elapsed:.2f}s): "{response_text}"')
                self._publish(response_text)
                self.get_logger().warn(f'Published to /tts_text → TTS should speak now')
            else:
                self.get_logger().warn('Generated empty response')

        except Exception as e:
            self.get_logger().error(f'Error generating response: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    def _publish(self, text):
        """Publish text to /tts_text."""
        msg = String()
        msg.data = text
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        llm_node = LLMNodeAPI()
        rclpy.spin(llm_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in Gemini LLM node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
