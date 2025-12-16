---
id: whisper-setup
title: "Lesson 1: Whisper Speech Recognition"
sidebar_position: 1
---

# Lesson 1: Whisper Speech Recognition

## What is Whisper?

OpenAI's Whisper: robust speech-to-text model trained on 680,000 hours of multilingual audio.

**Key features:**
- 99 languages supported
- Handles noisy environments
- Punctuation and capitalization
- Runs locally (no API key needed)

## Installation

```bash
pip install openai-whisper
sudo apt install ffmpeg
```

**Model sizes:**
- `tiny` (39M) - Fast, lower accuracy
- `base` (74M) - Balanced
- `small` (244M) - Good accuracy
- `medium` (769M) - Better accuracy
- `large` (1550M) - Best accuracy

For robotics: use `base` or `small` (real-time capable).

## Basic Usage

```python
import whisper

model = whisper.load_model("base")
result = model.transcribe("audio.wav")
print(result["text"])
```

## Microphone Configuration

**Install PyAudio:**
```bash
sudo apt install portaudio19-dev python3-pyaudio
pip install pyaudio
```

**List devices:**
```python
import pyaudio
p = pyaudio.PyAudio()
for i in range(p.get_device_count()):
    print(p.get_device_info_by_index(i))
```

**Record audio:**
```python
import pyaudio
import wave

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

p = pyaudio.PyAudio()
stream = p.open(format=FORMAT, channels=CHANNELS,
                rate=RATE, input=True, frames_per_buffer=CHUNK)

frames = []
for _ in range(0, int(RATE / CHUNK * 3)):  # 3 seconds
    frames.append(stream.read(CHUNK))

wf = wave.open("command.wav", 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(p.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(b''.join(frames))
```

## ROS 2 Integration

See `examples/whisper_ros_node.py` - publishes transcribed text to `/voice_command` topic.

**Launch node:**
```bash
ros2 run vla_demos whisper_ros_node
```

**Test:**
```bash
# Terminal 1: Start node
ros2 run vla_demos whisper_ros_node

# Terminal 2: Echo commands
ros2 topic echo /voice_command
```

Speak into microphone → see transcribed text.

## Tips

- Use push-to-talk button to reduce false triggers
- Filter commands with keyword ("Robot, go to kitchen")
- Set confidence threshold (reject low-confidence transcriptions)

## Next Lesson

Use transcribed commands with LLM for task planning.
