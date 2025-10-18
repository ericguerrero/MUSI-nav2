#!/bin/bash

echo "🛑 Stopping Nav2 Development Environment..."
docker compose down

echo "✅ Environment stopped."
echo ""
echo "Your work in ros2_ws/ and shared/ is preserved."
echo "Run ./run.sh to start again."
