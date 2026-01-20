#include "vicon_zmq/zmq_communicator.hpp"
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <memory>

static bool running = true;

void signal_handler(int signal)
{
    (void)signal;
    running = false;
}

int main(int argc, char** argv)
{
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Parse command line arguments
    std::string hostname = "127.0.0.1";
    unsigned int buffer_size = 200;
    std::string zmq_bind_address = "tcp://*:5555";

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--hostname" && i + 1 < argc) {
            hostname = argv[++i];
        } else if (arg == "--buffer-size" && i + 1 < argc) {
            buffer_size = std::stoi(argv[++i]);
        } else if (arg == "--zmq-address" && i + 1 < argc) {
            zmq_bind_address = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --hostname HOSTNAME      Vicon server hostname (default: 127.0.0.1)\n"
                      << "  --buffer-size SIZE       Buffer size (default: 200)\n"
                      << "  --zmq-address ADDRESS    ZMQ bind address (default: tcp://*:5555)\n"
                      << "  --help, -h               Show this help message\n";
            return 0;
        }
    }

    std::cout << "Starting Vicon ZMQ receiver..." << std::endl;
    std::cout << "  Vicon hostname: " << hostname << std::endl;
    std::cout << "  Buffer size: " << buffer_size << std::endl;
    std::cout << "  ZMQ bind address: " << zmq_bind_address << std::endl;

    auto communicator = std::make_shared<ZmqCommunicator>(hostname, buffer_size, zmq_bind_address);
    
    if (!communicator->connect()) {
        std::cerr << "Failed to connect to Vicon server" << std::endl;
        return 1;
    }

    std::cout << "Publishing Vicon data via ZMQ. Press Ctrl+C to stop." << std::endl;

    while (running) {
        communicator->get_frame();
    }

    std::cout << "Shutting down..." << std::endl;
    communicator->disconnect();
    
    return 0;
}

