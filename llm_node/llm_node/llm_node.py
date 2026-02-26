#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from llama_cpp import Llama
import threading
import os


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        # Declare parameters
        self.declare_parameter('model_path',
                             '/home/orin-robot/models/Llama-3.2-3B-Instruct-Q4_K_M.gguf')
        self.declare_parameter('system_prompt_path',
                             '/home/orin-robot/robot_ws/src/llm_node/prompts/sabis_robot_system.txt')
        self.declare_parameter('n_ctx', 2048)
        self.declare_parameter('n_gpu_layers', -1)  # Use GPU
        self.declare_parameter('n_batch', 512)  # Batch size for prompt processing
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('max_tokens', 200)  # Increased for better responses

        # Get parameters
        model_path = self.get_parameter('model_path').value
        system_prompt_path = self.get_parameter('system_prompt_path').value
        n_ctx = self.get_parameter('n_ctx').value
        n_gpu_layers = self.get_parameter('n_gpu_layers').value
        n_batch = self.get_parameter('n_batch').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value

        # Load system prompt
        self.system_prompt = self._load_system_prompt(system_prompt_path)

        # Initialize publisher and subscriber
        # Subscribe to STT output, publish to TTS input
        self.publisher = self.create_publisher(String, '/tts_text', 10)
        self.subscription = self.create_subscription(
            String,
            '/speech/text',
            self.speech_callback,
            10)

        # Subscribe to conversation reset signal
        self.reset_subscription = self.create_subscription(
            Bool,
            '/conversation/reset',
            self.reset_callback,
            10)

        # Conversation memory - stores chat history for current session
        self.conversation_history = []

        # Initialize LLM
        self.get_logger().info(f'Loading LLM model from {model_path}...')
        self.llm = None
        self.model_loaded = False
        self.lock = threading.Lock()

        # Load model in a separate thread to not block the executor
        threading.Thread(target=self._load_model,
                        args=(model_path, n_ctx, n_gpu_layers, n_batch),
                        daemon=True).start()

        self.get_logger().info('LLM node initialized. Subscribing to /speech/text and publishing to /tts_text')

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

    def _load_model(self, model_path, n_ctx, n_gpu_layers, n_batch):
        """Load the LLM model in a separate thread."""
        try:
            self.llm = Llama(
                model_path=model_path,
                n_ctx=n_ctx,
                n_gpu_layers=n_gpu_layers,
                n_batch=n_batch,
                use_mlock=False,  # Reduce memory overhead
                verbose=False
            )
            with self.lock:
                self.model_loaded = True
            self.get_logger().info('LLM model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load LLM model: {e}')

    def reset_callback(self, msg):
        """Handle conversation reset signal."""
        if msg.data:
            with self.lock:
                self.conversation_history = []
            self.get_logger().info('Conversation history cleared')

    def speech_callback(self, msg):
        """Handle incoming speech text and generate LLM response."""
        input_text = msg.data.strip()

        if not input_text:
            return

        self.get_logger().info(f'Received speech input: "{input_text}"')

        # Check if model is loaded
        with self.lock:
            if not self.model_loaded:
                self.get_logger().warn('Model not yet loaded, skipping request')
                return

        # Generate response in a separate thread to not block the executor
        threading.Thread(target=self._generate_response,
                        args=(input_text,),
                        daemon=True).start()

    def _generate_response(self, input_text):
        """Generate LLM response and publish it."""
        try:
            # Build prompt with conversation history
            # Format for Llama-3 (WITHOUT <|begin_of_text|> - llama.cpp adds it automatically)
            prompt = f"""<|start_header_id|>system<|end_header_id|>

{self.system_prompt}<|eot_id|>"""

            # Add conversation history
            with self.lock:
                for msg in self.conversation_history:
                    if msg['role'] == 'user':
                        prompt += f"""<|start_header_id|>user<|end_header_id|>

{msg['content']}<|eot_id|>"""
                    elif msg['role'] == 'assistant':
                        prompt += f"""<|start_header_id|>assistant<|end_header_id|>

{msg['content']}<|eot_id|>"""

            # Add current user message
            prompt += f"""<|start_header_id|>user<|end_header_id|>

{input_text}<|eot_id|><|start_header_id|>assistant<|end_header_id|>

"""

            # Generate response with lock to prevent concurrent calls (prevents segfault)
            self.get_logger().info('Generating LLM response...')
            with self.lock:
                response = self.llm(
                    prompt,
                    max_tokens=self.max_tokens,
                    temperature=self.temperature,
                    stop=["<|eot_id|>", "<|end_of_text|>"],
                    echo=False
                )

            # Extract the generated text
            response_text = response['choices'][0]['text'].strip()

            if response_text:
                self.get_logger().info(f'LLM response: "{response_text}"')

                # Add user message and assistant response to conversation history
                with self.lock:
                    self.conversation_history.append({
                        'role': 'user',
                        'content': input_text
                    })
                    self.conversation_history.append({
                        'role': 'assistant',
                        'content': response_text
                    })
                    # Log conversation length
                    self.get_logger().info(f'Conversation history: {len(self.conversation_history)} messages')

                # Publish response
                msg = String()
                msg.data = response_text
                self.publisher.publish(msg)
                self.get_logger().info(f'✓ Published to /tts_text: "{response_text}"')
            else:
                self.get_logger().warn('Generated empty response')

        except Exception as e:
            self.get_logger().error(f'Error generating response: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)

    try:
        llm_node = LLMNode()
        rclpy.spin(llm_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in LLM node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
