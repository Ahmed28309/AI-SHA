#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import google.generativeai as genai
import threading
import os


class LLMNodeGemini(Node):
    def __init__(self):
        super().__init__('llm_node')

        # Declare parameters
        self.declare_parameter('api_key', '')
        self.declare_parameter('model_name', 'gemini-2.5-flash')
        self.declare_parameter('system_prompt_path',
                             '/home/orin-robot/robot_ws/src/llm_node/prompts/sabis_robot_concise.txt')
        self.declare_parameter('temperature', 0.4)
        self.declare_parameter('max_tokens', 8192)

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
                max_output_tokens=self.max_tokens,
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

        self.get_logger().info('🚀 Gemini LLM node ready! Subscribing to /speech/text, publishing to /tts_text')

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

        self.get_logger().info(f'Received speech input: "{input_text}"')

        # Generate response in a separate thread to not block the executor
        threading.Thread(target=self._generate_response,
                        args=(input_text,),
                        daemon=True).start()

    def _generate_response(self, input_text):
        """Generate Gemini response and publish it."""
        try:
            start_time = __import__('time').time()
            self.get_logger().info('Generating Gemini response...')

            # Generate response
            response = self.model.generate_content(input_text)

            # Extract text
            response_text = response.text.strip()

            elapsed = __import__('time').time() - start_time

            if response_text:
                self.get_logger().info(f'Gemini response ({elapsed:.2f}s): "{response_text}"')

                # Publish response
                msg = String()
                msg.data = response_text
                self.publisher.publish(msg)
                self.get_logger().warn(f' Published to /tts_text → TTS should speak now')
            else:
                self.get_logger().warn('Generated empty response')

        except Exception as e:
            self.get_logger().error(f'Error generating response: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)

    try:
        llm_node = LLMNodeGemini()
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
