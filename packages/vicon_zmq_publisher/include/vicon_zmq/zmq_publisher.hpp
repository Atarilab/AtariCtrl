#ifndef ZMQ_PUBLISHER_HPP
#define ZMQ_PUBLISHER_HPP

#include <string>
#include <zmq.h>
#include <sstream>
#include <iomanip>
#include <cstring>

// Struct used to hold segment data to transmit to the Publisher class.
struct PositionStruct
{
    double translation[3];
    double rotation[4];
    std::string subject_name;
    std::string segment_name;
    std::string translation_type;
    unsigned int frame_number;
};

// Class that allows segment data to be published via ZMQ.
class ZmqPublisher
{
private:
    void* context_;
    void* socket_;

    // Serialize PositionStruct to JSON string
    std::string serialize_to_json(const PositionStruct& p);

public:
    bool is_ready = false;

    ZmqPublisher(const std::string& bind_address);

    // Publishes the given position via ZMQ with the specified topic name
    void publish(const PositionStruct& p, const std::string& topic_name);

    ~ZmqPublisher();
};

#endif // ZMQ_PUBLISHER_HPP

