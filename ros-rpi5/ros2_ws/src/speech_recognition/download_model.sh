#!/bin/bash
# Script to pre-download the best STT model for 2026

MODEL=${1:-large-v3-turbo}

echo "=========================================="
echo "  Downloading STT Model: $MODEL"
echo "=========================================="
echo ""

case $MODEL in
  large-v3-turbo)
    echo "Downloading large-v3-turbo (~1.6GB)"
    echo "  - Best model in 2026"
    echo "  - 7.75% WER (99%+ accuracy)"
    SIZE="1.6GB"
    ;;
  distil-large-v3)
    echo "Downloading distil-large-v3 (~756MB)"
    SIZE="756MB"
    ;;
  large-v3)
    echo "Downloading large-v3 (~3GB)"
    SIZE="3GB"
    ;;
  medium)
    echo "Downloading medium (~1.5GB)"
    SIZE="1.5GB"
    ;;
  small)
    echo "Downloading small (~466MB)"
    SIZE="466MB"
    ;;
  base)
    echo "Downloading base (~145MB)"
    SIZE="145MB"
    ;;
  tiny)
    echo "Downloading tiny (~75MB)"
    SIZE="75MB"
    ;;
  *)
    echo "Unknown model: $MODEL"
    echo "Available: large-v3-turbo, distil-large-v3, large-v3, medium, small, base, tiny"
    exit 1
    ;;
esac

echo ""
echo "This will download ~$SIZE from Hugging Face."
echo "Model will be cached in ~/.cache/huggingface/hub/"
echo ""
read -p "Continue? (y/N) " -n 1 -r
echo

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 1
fi

echo ""
echo "Downloading..."
echo ""

python3 << EOF
from faster_whisper import WhisperModel
import sys

try:
    print("Loading model '$MODEL'...")
    model = WhisperModel("$MODEL", device="cpu", compute_type="float16")
    print("")
    print("✓ Model '$MODEL' downloaded successfully!")
    print("✓ Cached in ~/.cache/huggingface/hub/")
    print("")
    print("You can now run the STT node with:")
    print("  ros2 run speech_recognition stt_node --ros-args -p model_size:=$MODEL")
    print("  OR")
    print("  ./run_stt.sh $MODEL")
except Exception as e:
    print(f"✗ Error downloading model: {e}")
    sys.exit(1)
EOF

echo ""
echo "Done!"
