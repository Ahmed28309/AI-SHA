#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import pyaudio
import numpy as np
from faster_whisper import WhisperModel
import os
import sys
import time
from collections import deque
from contextlib import contextmanager
import threading
import queue

@contextmanager
def suppress_alsa_errors():
    """Suppress ALSA error messages"""
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stderr = os.dup(2)
    sys.stderr.flush()
    os.dup2(devnull, 2)
    os.close(devnull)
    try:
        yield
    finally:
        os.dup2(old_stderr, 2)
        os.close(old_stderr)

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')

        # Parameters - Optimized for maximum accuracy
        self.declare_parameter('model_size', 'large-v3-turbo')  # Best model in 2026: large-v3-turbo, distil-large-v3, large-v3, medium, small, base, tiny
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('device_index', None)  # Auto-detect ReSpeaker
        self.declare_parameter('speech_energy_threshold', 200.0)  # Lower threshold for better detection
        self.declare_parameter('silence_duration', 0.8)  # Shorter silence for faster response
        self.declare_parameter('noise_floor_samples', 20)  # More samples for better noise estimation
        self.declare_parameter('language', 'en')  # Language code
        self.declare_parameter('beam_size', 5)  # Higher beam size for better accuracy (1-10, default 5)
        self.declare_parameter('best_of', 5)  # Number of candidates for beam search
        self.declare_parameter('patience', 1.5)  # Beam search patience factor
        self.declare_parameter('temperature', 0.0)  # 0 for deterministic, >0 for randomness

        model_size = self.get_parameter('model_size').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.device_index = self.get_parameter('device_index').value
        self.speech_threshold = self.get_parameter('speech_energy_threshold').value
        self.silence_duration = self.get_parameter('silence_duration').value
        self.language = self.get_parameter('language').value
        self.beam_size = self.get_parameter('beam_size').value
        self.best_of = self.get_parameter('best_of').value
        self.patience = self.get_parameter('patience').value
        self.temperature = self.get_parameter('temperature').value

        self.publisher = self.create_publisher(String, '/speech_rec', 10)
        self.is_robot_speaking = False
        self.create_subscription(Bool, '/robot/speaking', self.speaking_callback, 10)

        self.get_logger().info('Initializing Speech-to-Text Node with faster-whisper')

        # Load faster-whisper model
        try:
            self.get_logger().info(f'Loading faster-whisper model ({model_size})...')
            self.get_logger().info('Using large-v3-turbo: Best model in 2026 (6x faster than large-v3, 99% accuracy)')

            # Use float16 for better accuracy (Pi 5 can handle it)
            # int8 is faster but less accurate
            compute_type = "int8" if model_size in ['tiny', 'base'] else "float16"

            self.model = WhisperModel(
                model_size,
                device="cpu",
                compute_type=compute_type,
                num_workers=3,  # Pi 5 has 4 cores, use 3 for better throughput
                cpu_threads=4,  # Use all 4 cores for computation
                download_root=None,  # Use default cache
            )
            self.get_logger().info(f'Model loaded successfully (compute_type: {compute_type})')
        except Exception as e:
            self.get_logger().error(f'Failed to load faster-whisper: {e}')
            self.get_logger().error('Install with: pip install faster-whisper')
            return

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Auto-detect ReSpeaker or use specified device
        if self.device_index is None:
            self.device_index = self._find_respeaker_device()

        if self.device_index is None:
            self.get_logger().error('Could not find ReSpeaker device')
            return

        self.get_logger().info(f'Using audio device index: {self.device_index}')

        # VAD parameters
        self.chunk_size = 4000  # Frames per buffer
        self.noise_floor = deque(maxlen=self.get_parameter('noise_floor_samples').value)
        self.is_speaking = False
        self.silence_start = None
        self.audio_buffer = []  # Buffer to accumulate audio during speech

        # Threading for async transcription
        self.transcription_queue = queue.Queue()
        self.transcription_thread = threading.Thread(target=self._transcription_worker, daemon=True)
        self.transcription_thread.start()

        # Start continuous listening
        self.get_logger().info('Starting continuous speech recognition...')
        self.timer = self.create_timer(0.01, self.process_audio)  # Process at ~100Hz

        try:
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                input_device_index=self.device_index,
                frames_per_buffer=self.chunk_size,
                stream_callback=None
            )
            self.get_logger().info('Continuous listening active - speak naturally')
        except Exception as e:
            self.get_logger().error(f'Failed to open audio stream: {e}')
            return

    def _find_respeaker_device(self):
        """Find ReSpeaker device index"""
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            name = info.get('name', '').lower()
            if 'respeaker' in name or 'seeed' in name:
                self.get_logger().info(f'Found ReSpeaker: {info.get("name")}')
                return i
        # Fallback: try to find any USB audio device
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            if info.get('maxInputChannels', 0) > 0:
                self.get_logger().warn(f'Using fallback device: {info.get("name")}')
                return i
        return None

    def speaking_callback(self, msg):
        """Callback when robot starts/stops speaking"""
        self.is_robot_speaking = msg.data
        if msg.data:
            self.get_logger().info('Robot speaking - microphone muted')
            # Reset speech detection when robot starts speaking
            self.is_speaking = False
            self.silence_start = None
            self.audio_buffer = []

    def _transcription_worker(self):
        """Background thread for async transcription - OPTIMIZED FOR ACCURACY"""
        while True:
            try:
                audio_data = self.transcription_queue.get()
                if audio_data is None:  # Shutdown signal
                    break

                # Convert audio to float32 and normalize
                audio_float = audio_data.astype(np.float32) / 32768.0

                # Apply noise reduction preprocessing
                audio_float = self._denoise_audio(audio_float)

                self.get_logger().info(f'Transcribing {len(audio_float)/self.sample_rate:.2f}s audio with beam_size={self.beam_size}...')

                # Transcribe with faster-whisper - OPTIMIZED PARAMETERS FOR ACCURACY
                segments, info = self.model.transcribe(
                    audio_float,
                    language=self.language,
                    beam_size=self.beam_size,  # Higher beam size = better accuracy (default 5)
                    best_of=self.best_of,  # Number of candidates to consider
                    patience=self.patience,  # Beam search patience
                    temperature=self.temperature,  # 0 for deterministic
                    compression_ratio_threshold=2.4,  # Gzip compression ratio threshold
                    log_prob_threshold=-1.0,  # Log probability threshold
                    no_speech_threshold=0.6,  # Higher threshold for better speech detection
                    condition_on_previous_text=True,  # Use context for better accuracy
                    initial_prompt=None,  # Can add custom prompt for domain-specific vocab
                    word_timestamps=False,  # Disable for faster processing
                    prepend_punctuations="\"'([{-",
                    append_punctuations="\"'.,!?:)]}",
                    vad_filter=True,  # Use built-in Silero VAD for better accuracy
                    vad_parameters=dict(
                        threshold=0.5,  # VAD threshold
                        min_speech_duration_ms=250,  # Minimum speech duration
                        max_speech_duration_s=float('inf'),  # No limit on speech duration
                        min_silence_duration_ms=2000,  # Longer silence for better sentence boundaries
                        window_size_samples=1024,  # VAD window size
                        speech_pad_ms=400  # Padding around speech segments
                    ),
                    hallucination_silence_threshold=None,  # Disable to prevent cutting off speech
                    hotwords=None,  # Can add custom vocabulary here
                    language_detection_threshold=None,  # Trust the provided language
                    language_detection_segments=1
                )

                # Log detected language info
                self.get_logger().debug(f'Detected language: {info.language} (probability: {info.language_probability:.2f})')

                # Collect all segments
                text_parts = []
                for segment in segments:
                    text = segment.text.strip()
                    if text:
                        text_parts.append(text)
                        self.get_logger().debug(f'Segment: "{text}" (confidence: {segment.avg_logprob:.2f})')

                final_text = " ".join(text_parts).strip()

                if final_text:
                    self.get_logger().info(f'✓ Recognized: "{final_text}"')
                    self.publisher.publish(String(data=final_text))
                else:
                    self.get_logger().debug('Speech ended but no text recognized')

            except Exception as e:
                self.get_logger().error(f'Transcription error: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())

    def _denoise_audio(self, audio):
        """Simple noise reduction using spectral gating"""
        try:
            # Calculate noise profile from first 0.5 seconds (assumed to be quieter)
            noise_sample_size = min(int(0.5 * self.sample_rate), len(audio) // 4)
            if noise_sample_size > 0:
                noise_profile = np.abs(audio[:noise_sample_size]).mean()
                # Reduce very quiet samples (likely noise)
                threshold = noise_profile * 1.5
                audio = np.where(np.abs(audio) < threshold, audio * 0.3, audio)
            return audio
        except:
            return audio  # Return original if denoising fails

    def _calculate_energy(self, audio_data):
        """Calculate RMS energy of audio data"""
        return np.sqrt(np.mean(audio_data.astype(np.float32)**2))

    def _update_noise_floor(self, energy):
        """Update dynamic noise floor estimation"""
        self.noise_floor.append(energy)

    def _get_dynamic_threshold(self):
        """Calculate dynamic threshold based on noise floor - IMPROVED"""
        if len(self.noise_floor) > 0:
            avg_noise = np.mean(self.noise_floor)
            std_noise = np.std(self.noise_floor)
            # Speech threshold is mean + 2*std (captures 95% of noise variations)
            dynamic = avg_noise + (2.5 * std_noise)
            # Use the higher of static threshold or dynamic threshold
            return max(self.speech_threshold, dynamic)
        return self.speech_threshold

    def process_audio(self):
        """Continuous audio processing with VAD"""
        if not hasattr(self, 'stream') or self.stream is None:
            return

        # Don't process audio when robot is speaking
        if self.is_robot_speaking:
            # Drain the buffer to prevent overflow
            try:
                if self.stream.get_read_available() > 0:
                    self.stream.read(self.stream.get_read_available(), exception_on_overflow=False)
            except:
                pass
            return

        try:
            # Read available audio data
            available = self.stream.get_read_available()
            if available < self.chunk_size:
                return

            data = self.stream.read(self.chunk_size, exception_on_overflow=False)
            audio_data = np.frombuffer(data, dtype=np.int16)
            energy = self._calculate_energy(audio_data)

            # Calculate dynamic threshold
            dynamic_threshold = self._get_dynamic_threshold()

            # Voice Activity Detection
            if energy > dynamic_threshold:
                # Speech detected
                if not self.is_speaking:
                    self.is_speaking = True
                    self.get_logger().info(f'Speech started (energy: {energy:.1f}, threshold: {dynamic_threshold:.1f})')
                    self.audio_buffer = []  # Start fresh buffer

                # Reset silence timer
                self.silence_start = None

                # Buffer audio data
                self.audio_buffer.append(audio_data)

            else:
                # Update noise floor when no speech detected
                if not self.is_speaking:
                    self._update_noise_floor(energy)

                # Silence detected while speaking
                if self.is_speaking:
                    if self.silence_start is None:
                        self.silence_start = time.time()

                    # Check if silence duration exceeded
                    silence_time = time.time() - self.silence_start
                    if silence_time >= self.silence_duration:
                        # End of speech - transcribe buffered audio
                        if len(self.audio_buffer) > 0:
                            full_audio = np.concatenate(self.audio_buffer)
                            # Queue for async transcription
                            self.transcription_queue.put(full_audio)
                            self.get_logger().info(f'Speech ended, transcribing {len(full_audio)/self.sample_rate:.2f}s of audio...')
                        else:
                            self.get_logger().debug('Speech ended but buffer empty')

                        # Reset state
                        self.is_speaking = False
                        self.silence_start = None
                        self.audio_buffer = []

        except IOError as e:
            # Buffer overflow - just skip this iteration
            self.get_logger().debug(f'Buffer overflow, skipping frame')
        except Exception as e:
            self.get_logger().error(f'Audio processing error: {e}')

    def __del__(self):
        """Cleanup audio resources"""
        # Shutdown transcription thread
        if hasattr(self, 'transcription_queue'):
            self.transcription_queue.put(None)
        if hasattr(self, 'transcription_thread') and self.transcription_thread.is_alive():
            self.transcription_thread.join(timeout=2.0)

        if hasattr(self, 'stream') and self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'audio') and self.audio is not None:
            self.audio.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
