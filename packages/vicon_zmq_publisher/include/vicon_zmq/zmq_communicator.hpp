#ifndef ZMQ_COMMUNICATOR_HPP
#define ZMQ_COMMUNICATOR_HPP

#include "DataStreamClient.h"
#include "zmq_publisher.hpp"
#include <iostream>
#include <string>
#include <unistd.h>

using namespace std;

// Main class for Vicon data receiver with ZMQ publishing
class ZmqCommunicator
{
private:
    ViconDataStreamSDK::CPP::Client vicon_client;
    string hostname;
    unsigned int buffer_size;
    string zmq_bind_address;
    ZmqPublisher* zmq_publisher_;  // Single publisher for all topics

public:
    ZmqCommunicator(const string& hostname = "127.0.0.1", 
                    unsigned int buffer_size = 200,
                    const string& zmq_bind_address = "tcp://*:5555");

    ~ZmqCommunicator();

    // Initialises the connection to the DataStream server
    bool connect();

    // Stops the current connection to a DataStream server (if any).
    bool disconnect();

    // Main loop that request frames from the currently connected DataStream server and send the 
    // received segment data to the ZmqPublisher class.
    void get_frame();
};

#endif // ZMQ_COMMUNICATOR_HPP

