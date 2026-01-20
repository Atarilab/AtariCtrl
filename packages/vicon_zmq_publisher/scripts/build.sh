#!/bin/bash
# Build script for Vicon ZMQ standalone package

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

echo "Building Vicon ZMQ standalone package..."

# Check if required libraries are installed
if ! pkg-config --exists libzmq; then
    echo "Error: ZMQ library not found. Please install it:"
    echo "  sudo apt-get install libzmq3-dev"
    exit 1
fi

# Check if Vicon SDK library exists
if [ ! -f "libs/libViconDataStreamSDK_CPP.so" ]; then
    echo "Warning: Vicon SDK library not found at libs/libViconDataStreamSDK_CPP.so"
    echo "Please copy the Vicon SDK libraries to the libs/ directory"
    exit 1
fi

# Create build directory
mkdir -p build
cd build

# Run CMake
cmake ..

# Build
make -j$(nproc)

echo ""
echo "Build complete! Executable is at: build/vicon_zmq_client"
echo ""
echo "To install system-wide:"
echo "  cd build && sudo make install"
echo ""
echo "To run:"
echo "  ./build/vicon_zmq_client --hostname <vicon_server_ip>"

