#!/usr/bin/env bash

# ESP32 Build Script - Simple version
# Usage: ./esp32_build.sh [build|flash|monitor|clean|fullclean]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}/i2c_basic"
COMMAND="${1:-build}"
PORT="${2:-/dev/ttyUSB1}"

# Source ESP-IDF
if [ -z "$IDF_PATH" ]; then
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        . "$HOME/esp/esp-idf/export.sh"
    else
        echo "Error: ESP-IDF not found at ~/esp/esp-idf/"
        echo "Please install ESP-IDF or source it manually"
        exit 1
    fi
fi

cd "$PROJECT_DIR"

case "$COMMAND" in
    build)
        idf.py build
        ;;
    flash)
        idf.py -p "$PORT" flash
        ;;
    monitor)
        idf.py -p "$PORT" flash monitor
        ;;
    clean)
        idf.py clean
        ;;
    fullclean)
        idf.py fullclean
        ;;
    *)
        echo "Usage: $0 [build|flash|monitor|clean|fullclean] [port]"
        echo "Default port: $PORT"
        exit 1
        ;;
esac

