# Vicon ZMQ Standalone Package

## Package Overview

This is a standalone package for receiving Vicon motion capture data and publishing it via ZeroMQ. The package is completely independent and does not require ROS2, colcon, or any ROS dependencies.

## Package Structure

```
vicon-zmq-standalone/
├── CMakeLists.txt              # CMake build configuration
├── README.md                    # Main documentation
├── QUICKSTART.md                # Quick start guide
├── INSTALL.md                   # Detailed installation guide
├── PACKAGE_INFO.md              # This file
├── .gitignore                   # Git ignore file
├── include/
│   └── vicon_zmq/
│       ├── zmq_publisher.hpp    # ZMQ publisher header
│       └── zmq_communicator.hpp # ZMQ communicator header
├── src/
│   ├── zmq_publisher.cpp        # ZMQ publisher implementation
│   ├── zmq_communicator.cpp     # ZMQ communicator implementation
│   └── main.cpp                 # Main entry point
├── libs/                        # Vicon SDK libraries and headers
│   ├── libViconDataStreamSDK_CPP.so
│   ├── libboost_*.so            # Boost libraries
│   ├── DataStreamClient.h
│   └── IDataStreamClientBase.h
└── scripts/
    ├── build.sh                 # Build script
    ├── install.sh               # Installation script
    └── vicon_zmq_subscriber.py   # Python subscriber script
```

## Features

- ✅ **Standalone**: No ROS2 or colcon dependencies
- ✅ **ZMQ Publishing**: ZeroMQ-based message publishing
- ✅ **JSON Format**: Human-readable JSON message format
- ✅ **Multi-topic**: Single socket for all subjects/segments
- ✅ **Python Subscriber**: Included subscriber script
- ✅ **Installable**: Can be installed system-wide
- ✅ **Portable**: Self-contained package

## Quick Usage

### Build
```bash
./scripts/build.sh
```

### Run Publisher
```bash
./build/vicon_zmq_client --hostname <vicon_server_ip>
```

### Run Subscriber
```bash
./scripts/vicon_zmq_subscriber.py
```

### Install System-Wide
```bash
./scripts/install.sh
```

## Dependencies

### System Dependencies
- libzmq3-dev
- cmake
- build-essential
- libboost-all-dev

### Vicon SDK Files (provided in libs/)
- Vicon DataStream SDK libraries
- Vicon DataStream SDK headers

## Message Format

Messages are published as multi-part ZMQ messages:
1. **Topic**: `subject_name/segment_name`
2. **Data**: JSON string with position information

Example JSON:
```json
{
  "subject_name": "MySubject",
  "segment_name": "MySegment",
  "translation": [1.234, 5.678, 9.012],
  "rotation": [0.0, 0.0, 0.0, 1.0],
  "translation_type": "Global",
  "frame_number": 12345
}
```

## Installation Locations

After installation with `./scripts/install.sh`:

- Executable: `/usr/local/bin/vicon_zmq_client`
- Headers: `/usr/local/include/vicon_zmq/`
- Libraries: `/usr/local/lib/vicon_zmq/`
- Python Script: `/usr/local/bin/vicon_zmq_subscriber.py`

## License

This package uses the Vicon DataStream SDK, which is subject to Vicon's license terms.

