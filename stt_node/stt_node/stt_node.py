#!/usr/bin/env python3
"""
STT Node - Optimized Faster-Whisper (GPU Accelerated)
Uses distil-large-v3 for maximum accuracy and speed
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import sounddevice as sd
import numpy as np
import threading
import queue
import time as system_time
from faster_whisper import WhisperModel
import subprocess
import time as time_module


class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')

        # Kill PulseAudio to allow direct ReSpeaker access
        try:
            subprocess.run(['systemctl', '--user', 'stop', 'pulseaudio.socket', 'pulseaudio.service'], 
                         stderr=subprocess.DEVNULL, check=False)
            subprocess.run(['pulseaudio', '--kill'], stderr=subprocess.DEVNULL, check=False)
            time_module.sleep(1)
            self.get_logger().info('Stopped PulseAudio for direct ReSpeaker access')
        except Exception as e:
            self.get_logger().warn(f'Could not stop PulseAudio: {e}')

        # Optimized parameters
        self.sample_rate = 16000
        self.device_channels = 1  # Start with mono for compatibility
        self.channels = 1  # Process as mono
        self.chunk_duration = 0.5  # Faster chunking
        self.silence_threshold = 0.03  # Higher threshold to reduce noise pickup
        self.device_index = None
        self.respeaker_found = False

        # Publisher
        self.text_pub = self.create_publisher(String, '/speech/text', 10)

        # Subscribe to /robot/speaking to mute mic during TTS playback
        self.robot_is_speaking = False
        self.speaking_sub = self.create_subscription(
            Bool,
            '/robot/speaking',
            self.speaking_callback,
            10
        )

        # Audio queue
        self.audio_queue = queue.Queue()

        # Model loading
        self.model = None
        self.model_loaded = threading.Event()

        # Find ReSpeaker
        self._find_respeaker()

        # Load optimized model
        self.get_logger().info('Loading Faster-Whisper base model...')
        threading.Thread(target=self._load_model, daemon=True).start()

        # Start audio processing
        threading.Thread(target=self._process_audio, daemon=True).start()

        # Start audio stream
        self._start_stream()

    def speaking_callback(self, msg):
        """Handle /robot/speaking messages to mute mic during TTS"""
        self.robot_is_speaking = msg.data
        if self.robot_is_speaking:
            self.get_logger().info('🔇 Mic muted - Robot is speaking', throttle_duration_sec=1.0)
        else:
            self.get_logger().info('🎤 Mic active - Listening', throttle_duration_sec=1.0)

    def _find_respeaker(self):
        """Auto-detect ReSpeaker mic array"""
        devices = sd.query_devices()

        for idx, device in enumerate(devices):
            name = str(device.get('name', '')).lower()
            if 'respeaker' in name or 'seeed' in name:
                # Verify device has input channels
                max_input = device.get('max_input_channels', 0)
                self.respeaker_found = True
                self.device_index = idx  # Use the ReSpeaker device directly
                self.device_channels = 1  # Use mono (auto-mixed from 6 channels)
                self.get_logger().info(f'Found ReSpeaker: {device.get("name")} (device {idx})')
                self.get_logger().info(f'Device capabilities: {max_input} input channels, {device.get("default_samplerate", 0)}Hz default')
                self.get_logger().info(f'Using device {idx} in mono mode (1ch) for speech recognition')
                return

        self.get_logger().warn('ReSpeaker not found, using system default mic')
        self.device_index = None
        self.device_channels = 1

    def _load_model(self):
        """Load optimized Faster-Whisper model"""
        try:
            # Try CUDA first, fallback to CPU if not available
            import ctranslate2
            has_cuda = ctranslate2.get_cuda_device_count() > 0

            if has_cuda:
                self.get_logger().info('Loading on GPU with optimized settings...')
                device = "cuda"
                compute_type = "float16"
            else:
                self.get_logger().warn('CUDA not available, using CPU with int8 quantization...')
                device = "cpu"
                compute_type = "int8"

            # Use "base" model - well supported on CPU with good accuracy
            self.model = WhisperModel(
                "base",  # ~150MB, good accuracy, fast on CPU
                device=device,
                compute_type=compute_type,
                num_workers=2
            )

            # Quick warmup
            dummy_audio = np.zeros(self.sample_rate * 2, dtype=np.float32)
            _ = list(self.model.transcribe(
                dummy_audio,
                language="en",
                beam_size=1,
                vad_filter=False
            ))

            self.model_loaded.set()
            if has_cuda:
                self.get_logger().info('✓ Whisper base ready on GPU (high accuracy, fast inference)')
                self.get_logger().info('Expected latency: 0.5-1.5s per utterance')
            else:
                self.get_logger().info('✓ Whisper base ready on CPU (int8 quantization)')
                self.get_logger().info('Expected latency: 1-3s per utterance (CPU mode)')

        except Exception as e:
            self.get_logger().error(f'Model load failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _audio_callback(self, indata, frames, time_info, status):
        """Callback for audio stream - handle mono or multi-channel"""
        if status:
            self.get_logger().warn(f'Audio status: {status}', throttle_duration_sec=5.0)

        # Convert to mono if multi-channel
        if len(indata.shape) > 1 and indata.shape[1] > 1:
            mono_audio = np.mean(indata, axis=1, keepdims=True)
        else:
            mono_audio = indata.reshape(-1, 1) if len(indata.shape) == 1 else indata

        self.audio_queue.put(mono_audio.copy())

    def _start_stream(self):
        """Start continuous audio capture"""
        chunk_samples = int(self.sample_rate * self.chunk_duration)

        try:
            # Try opening with specified channels
            self.stream = sd.InputStream(
                device=self.device_index,
                channels=self.device_channels,
                samplerate=self.sample_rate,
                blocksize=chunk_samples,
                callback=self._audio_callback
            )
            self.stream.start()
            self.get_logger().info(f'Recording: {self.sample_rate}Hz, {self.device_channels}ch device -> 1ch mono')

        except Exception as e:
            self.get_logger().error(f'Stream start failed: {e}')
            # Try fallback to default device
            try:
                self.get_logger().warn('Trying default audio device...')
                self.device_channels = 1
                self.stream = sd.InputStream(
                    samplerate=self.sample_rate,
                    channels=1,
                    blocksize=chunk_samples,
                    callback=self._audio_callback
                )
                self.stream.start()
                self.get_logger().info(f'Using default device: {self.sample_rate}Hz, 1ch')
            except Exception as e2:
                self.get_logger().error(f'All audio devices failed: {e2}')

    def _is_speech(self, audio_chunk):
        """Optimized voice activity detection"""
        rms = np.sqrt(np.mean(audio_chunk**2))
        return rms > self.silence_threshold

    def _process_audio(self):
        """Process audio chunks from queue"""
        buffer = []
        silence_count = 0
        max_buffer_chunks = 30  # ~15 seconds
        min_speech_chunks = 3   # Require at least 1.5 sec of speech to reduce false triggers
        required_silence_chunks = 3  # 1.5 seconds silence

        while rclpy.ok():
            try:
                chunk = self.audio_queue.get(timeout=0.5)

                if self._is_speech(chunk):
                    buffer.append(chunk)
                    silence_count = 0

                    if len(buffer) >= max_buffer_chunks:
                        self._transcribe_buffer(buffer)
                        buffer = []
                else:
                    if buffer:
                        silence_count += 1
                        if silence_count >= required_silence_chunks and len(buffer) >= min_speech_chunks:
                            self._transcribe_buffer(buffer)
                            buffer = []
                            silence_count = 0

            except queue.Empty:
                if buffer and len(buffer) >= min_speech_chunks:
                    self._transcribe_buffer(buffer)
                    buffer = []
                continue
            except Exception as e:
                self.get_logger().error(f'Process error: {e}', throttle_duration_sec=5.0)

    def _transcribe_buffer(self, buffer):
        """Transcribe with optimized settings"""
        # Skip transcription if robot is speaking (prevents feedback loop)
        if self.robot_is_speaking:
            self.get_logger().debug('Skipping transcription - robot is speaking', throttle_duration_sec=2.0)
            return

        if not self.model_loaded.is_set():
            return

        try:
            # Concatenate buffer
            audio = np.concatenate(buffer).flatten()

            # Convert to float32
            if audio.dtype != np.float32:
                audio = audio.astype(np.float32)

            # Transcribe with optimized settings
            start_time = system_time.time()

            segments, info = self.model.transcribe(
                audio,
                language="en",
                beam_size=5,  # Higher beam for better accuracy
                best_of=5,    # Consider more candidates
                temperature=0.0,  # Deterministic
                vad_filter=True,  # Built-in VAD
                vad_parameters=dict(
                    min_silence_duration_ms=500,  # Longer silence required
                    threshold=0.6,  # Higher VAD threshold to reduce false positives
                    min_speech_duration_ms=250  # Require longer speech segments
                ),
                condition_on_previous_text=False,  # Faster, more accurate for short segments
                compression_ratio_threshold=2.4,
                log_prob_threshold=-1.0,  # More confident (was -0.8)
                no_speech_threshold=0.6,  # Higher to better reject silence (was 0.5)
                initial_prompt=None  # Removed to prevent prompt leakage
            )

            # Collect all segments
            text_segments = []
            for segment in segments:
                text_segments.append(segment.text)

            elapsed = system_time.time() - start_time
            text = ' '.join(text_segments).strip()

            if text:
                # Publish
                msg = String()
                msg.data = text
                self.text_pub.publish(msg)

                self.get_logger().info(f'"{text}" ({elapsed:.2f}s)')

        except Exception as e:
            self.get_logger().error(f'Transcription failed: {e}', throttle_duration_sec=5.0)

    def destroy_node(self):
        """Cleanup on shutdown"""
        if hasattr(self, 'stream'):
            self.stream.stop()
            self.stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
