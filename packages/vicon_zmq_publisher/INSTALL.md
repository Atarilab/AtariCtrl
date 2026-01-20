# Installation Guide

## System Requirements

- Linux (Ubuntu 18.04+ recommended)
- CMake 3.5 or higher
- C++14 compatible compiler (GCC 5+ or Clang 3.5+)
- ZMQ library (libzmq3-dev)
- Boost libraries (libboost-all-dev)

## Step 1: Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
    libzmq3-dev \
    cmake \
    build-essential \
    libboost-all-dev
```

## Step 2: Prepare Vicon SDK Files

Copy the following files from your Vicon DataStream SDK installation to the `libs/` directory:

### Required Libraries:
- `libViconDataStreamSDK_CPP.so`
- `libboost_system-mt-x64.so.1.75.0` (or compatible version)
- `libboost_thread-mt-x64.so.1.75.0`
- `libboost_timer-mt-x64.so.1.75.0`
- `libboost_chrono-mt-x64.so.1.75.0`

### Required Headers:
- `DataStreamClient.h`
- `IDataStreamClientBase.h`

These files are typically found in the Vicon DataStream SDK installation directory.

## Step 3: Build

```bash
./scripts/build.sh
```

Or manually:
```bash
mkdir build
cd build
cmake ..
make
```

## Step 4: Install (Optional)

### System-Wide Installation

```bash
./scripts/install.sh
```

This installs to `/usr/local` by default.

### Custom Installation Path

```bash
./scripts/install.sh /opt/vicon_zmq
```

### Manual Installation

```bash
cd build
sudo make install
```

## Installation Locations

After installation, files are placed in:

- **Executable**: `/usr/local/bin/vicon_zmq_client`
- **Headers**: `/usr/local/include/vicon_zmq/`
- **Libraries**: `/usr/local/lib/vicon_zmq/`
- **Python Script**: `/usr/local/bin/vicon_zmq_subscriber.py`

## Post-Installation

After installation, you may need to update the library cache:

```bash
sudo ldconfig
```

## Verification

Test the installation:

```bash
vicon_zmq_client --help
vicon_zmq_subscriber.py --help
```

## Uninstallation

To uninstall:

```bash
cd build
sudo xargs rm < install_manifest.txt
sudo ldconfig
```

