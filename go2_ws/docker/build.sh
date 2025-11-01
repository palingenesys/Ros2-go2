#!/bin/bash
set -e

cd "$(dirname "$0")"

IMAGE_NAME="go2_humble"

echo "[INFO] Building Docker image..."
docker build -t $IMAGE_NAME .
