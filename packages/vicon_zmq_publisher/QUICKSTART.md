# Quick Start Guide

## Prerequisites

1. Install system dependencies:
```bash
sudo apt-get update
sudo apt-get install -y libzmq3-dev cmake build-essential libboost-all-dev
```

2. Copy Vicon SDK files to `libs/` directory:
   - `libViconDataStreamSDK_CPP.so`
   - `libboost_system-mt-x64.so.1.75.0` (or compatible)
   - `libboost_thread-mt-x64.so.1.75.0`
   - `libboost_timer-mt-x64.so.1.75.0`
   - `libboost_chrono-mt-x64.so.1.75.0`
   - `DataStreamClient.h`
   - `IDataStreamClientBase.h`

## Build

```bash
./scripts/build.sh
```

Or manually:
```bash
mkdir build && cd build
cmake ..
make
```

## Install (Optional)

To install system-wide:
```bash
./scripts/install.sh
```

Or to a custom location:
```bash
./scripts/install.sh /opt/vicon_zmq
```

## Run

### Publisher

```bash
./build/vicon_zmq_client --hostname <vicon_server_ip>
```

### Subscriber

In another terminal:
```bash
./scripts/vicon_zmq_subscriber.py
```

Or if installed:
```bash
vicon_zmq_subscriber.py
```

## Example

Terminal 1 (Publisher):
```bash
./build/vicon_zmq_client --hostname 192.168.123.100
```

Terminal 2 (Subscriber):
```bash
./scripts/vicon_zmq_subscriber.py --zmq-address tcp://localhost:5555
```

