#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# ROOT_DIR is one level up from scripts/
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

TARGET_DIRS=(
    "$ROOT_DIR/Log/Colmap/images"
    "$ROOT_DIR/Log/Colmap/sparse/0"
    "$ROOT_DIR/Log/PCD"
)

for dir in "${TARGET_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        rm -rf "$dir"
        echo "Removed: $dir"
    fi
done

for dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        echo "Created: $dir"
    else
        echo "Exists: $dir"
    fi
done

