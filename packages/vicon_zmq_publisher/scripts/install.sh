#!/bin/bash
# Installation script for Vicon ZMQ standalone package

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# Default install prefix
INSTALL_PREFIX="${1:-/usr/local}"

echo "Installing Vicon ZMQ standalone package to $INSTALL_PREFIX"

# Check if built
if [ ! -f "build/vicon_zmq_client" ]; then
    echo "Error: Package not built. Please run build first:"
    echo "  ./scripts/build.sh"
    exit 1
fi

# Install using CMake
cd build
cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" ..
sudo make install

# Install Vicon SDK libraries to system library path
if [ -d "libs" ]; then
    echo "Installing Vicon SDK libraries..."
    sudo cp ../libs/*.so /usr/lib/ 2>/dev/null || true
    sudo ldconfig
fi

echo ""
echo "Installation complete!"
echo ""
echo "Installed files:"
echo "  Executable: $INSTALL_PREFIX/bin/vicon_zmq_client"
echo "  Headers: $INSTALL_PREFIX/include/vicon_zmq/"
echo "  Libraries: $INSTALL_PREFIX/lib/vicon_zmq/"
echo "  Python script: $INSTALL_PREFIX/bin/vicon_zmq_subscriber.py"
echo ""
echo "You can now run:"
echo "  vicon_zmq_client --hostname <vicon_server_ip>"

