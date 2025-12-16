#!/usr/bin/env python3
"""
Whisper ROS 2 Node

Continuously listens to microphone and publishes transcribed text to /voice_command.

Usage:
  ros2 run vla_demos whisper_ros_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio
import wave
import tempfile
import os


class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Publishers
        self.command_pub = self.create_publisher(String, '/voice_command', 10)

        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('record_seconds', 3)
        self.declare_parameter('keyword', 'robot')  # Activation keyword

        model_size = self.get_parameter('model_size').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.record_seconds = self.get_parameter('record_seconds').value
        self.keyword = self.get_parameter('keyword').value.lower()

        # Load Whisper model
        self.get_logger().info(f"Loading Whisper model: {model_size}")
        self.model = whisper.load_model(model_size)
        self.get_logger().info("Whisper model loaded")

        # Audio setup
        self.audio = pyaudio.PyAudio()
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1

        # Start listening timer
        self.timer = self.create_timer(0.1, self.listen_callback)

        self.get_logger().info(f"Whisper node started. Say '{self.keyword}' to activate.")

    def record_audio(self):
        """Record audio from microphone."""
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        self.get_logger().info("Recording...")
        frames = []

        for _ in range(0, int(self.sample_rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        # Save to temporary file
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".wav")
        wf = wave.open(temp_file.name, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.audio.get_sample_size(self.format))
        wf.setframerate(self.sample_rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return temp_file.name

    def transcribe_audio(self, audio_file):
        """Transcribe audio using Whisper."""
        result = self.model.transcribe(audio_file, language='en')
        return result['text'].strip()

    def listen_callback(self):
        """Continuously listen for commands."""
        try:
            # Record audio
            audio_file = self.record_audio()

            # Transcribe
            text = self.transcribe_audio(audio_file)
            os.unlink(audio_file)  # Clean up temp file

            if not text:
                return

            self.get_logger().info(f"Transcribed: {text}")

            # Check for activation keyword
            if self.keyword in text.lower():
                # Remove keyword from command
                command = text.lower().replace(self.keyword, "").strip()

                if command:
                    self.get_logger().info(f"Command: {command}")
                    msg = String()
                    msg.data = command
                    self.command_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def destroy_node(self):
        """Clean up audio resources."""
        self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
