# Vicon ZMQ Standalone Package

A standalone package for receiving Vicon motion capture data and publishing it via ZMQ. This package has no dependencies on ROS2 or colcon.

## Features

- **Standalone**: No ROS2 or colcon required
- **ZMQ Publishing**: Publishes Vicon data via ZeroMQ
- **JSON Format**: Human-readable JSON message format
- **Multi-topic Support**: Publishes all subjects/segments through a single ZMQ socket
- **Python Subscriber**: Included Python script for subscribing to data

## Requirements

### System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
    libzmq3-dev \
    cmake \
    build-essential \
    libboost-all-dev
```

### Vicon SDK Libraries

The Vicon DataStream SDK libraries must be provided. Copy the following files to the `libs/` directory:

- `libViconDataStreamSDK_CPP.so`
- `libboost_system-mt-x64.so.1.75.0` (or compatible version)
- `libboost_thread-mt-x64.so.1.75.0`
- `libboost_timer-mt-x64.so.1.75.0`
- `libboost_chrono-mt-x64.so.1.75.0`

Also copy the Vicon SDK headers:
- `DataStreamClient.h`
- `IDataStreamClientBase.h`

## Building

### Build from Source

```bash
mkdir build
cd build
cmake ..
make
```

Or use the provided build script:

```bash
./scripts/build.sh
```

The executable will be created at `build/vicon_zmq_client`.

## Installation

### Install System-Wide

```bash
cd build
sudo make install
```

This will install:
- Executable: `/usr/local/bin/vicon_zmq_client`
- Headers: `/usr/local/include/vicon_zmq/`
- Libraries: `/usr/local/lib/vicon_zmq/`
- Python script: `/usr/local/bin/vicon_zmq_subscriber.py`

### Install to Custom Location

```bash
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/vicon_zmq ..
make
sudo make install
```

## Usage

### Running the Publisher

```bash
vicon_zmq_client --hostname 192.168.123.100
```

Or with custom options:

```bash
vicon_zmq_client \
    --hostname 192.168.123.100 \
    --buffer-size 200 \
    --zmq-address tcp://*:5555
```

### Command Line Options

- `--hostname HOSTNAME`: Vicon server hostname (default: 127.0.0.1)
- `--buffer-size SIZE`: Buffer size (default: 200)
- `--zmq-address ADDRESS`: ZMQ bind address (default: tcp://*:5555)
- `--help, -h`: Show help message

### Running the Python Subscriber

```bash
vicon_zmq_subscriber.py
```

Or with options:

```bash
# Subscribe to all topics
vicon_zmq_subscriber.py --zmq-address tcp://localhost:5555

# Subscribe to a specific topic prefix
vicon_zmq_subscriber.py --topic-filter "MySubject/"

# Output raw JSON
vicon_zmq_subscriber.py --json
```

## Message Format

The ZMQ publisher sends multi-part messages:
1. **Topic name**: `subject_name/segment_name` (e.g., "MySubject/MySegment")
2. **JSON data**: Position data serialized as JSON

### JSON Message Structure

```json
{
  "subject_name": "MySubject",
  "segment_name": "MySegment",
  "translation": [x, y, z],
  "rotation": [x, y, z, w],
  "translation_type": "Global",
  "frame_number": 12345
}
```

## Directory Structure

```
vicon-zmq-standalone/
├── CMakeLists.txt          # Build configuration
├── README.md               # This file
├── include/
│   └── vicon_zmq/
│       ├── zmq_publisher.hpp
│       └── zmq_communicator.hpp
├── src/
│   ├── zmq_publisher.cpp
│   ├── zmq_communicator.cpp
│   └── main.cpp
├── libs/                   # Vicon SDK libraries and headers
│   ├── libViconDataStreamSDK_CPP.so
│   ├── libboost_*.so
│   ├── DataStreamClient.h
│   └── IDataStreamClientBase.h
└── scripts/
    ├── build.sh            # Build script
    ├── install.sh          # Installation script
    └── vicon_zmq_subscriber.py  # Python subscriber
```

## Troubleshooting

### Library Loading Errors

If you get errors about missing libraries:

1. Make sure the Vicon SDK libraries are in the `libs/` directory
2. Install the libraries system-wide:
   ```bash
   sudo cp libs/*.so /usr/lib/
   sudo ldconfig
   ```
3. Or set LD_LIBRARY_PATH:
   ```bash
   export LD_LIBRARY_PATH=$PWD/libs:$LD_LIBRARY_PATH
   ```

### Connection Issues

If the publisher can't connect to the Vicon server:

1. Verify the Vicon server is running
2. Check the hostname/IP address
3. Ensure network connectivity
4. Check firewall settings

## License

This package uses the Vicon DataStream SDK, which is subject to Vicon's license terms.

