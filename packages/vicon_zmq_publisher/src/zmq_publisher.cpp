#include "vicon_zmq/zmq_publisher.hpp"
#include <iostream>
#include <cstring>
#include <cerrno>

ZmqPublisher::ZmqPublisher(const std::string& bind_address)
    : context_(nullptr), socket_(nullptr)
{
    context_ = zmq_ctx_new();
    if (!context_) {
        std::cerr << "Failed to create ZMQ context" << std::endl;
        is_ready = false;
        return;
    }

    socket_ = zmq_socket(context_, ZMQ_PUB);
    if (!socket_) {
        std::cerr << "Failed to create ZMQ socket: " << zmq_strerror(zmq_errno()) << std::endl;
        zmq_ctx_destroy(context_);
        context_ = nullptr;
        is_ready = false;
        return;
    }

    int rc = zmq_bind(socket_, bind_address.c_str());
    if (rc != 0) {
        std::cerr << "Failed to bind ZMQ socket to " << bind_address 
                  << ": " << zmq_strerror(zmq_errno()) << std::endl;
        zmq_close(socket_);
        zmq_ctx_destroy(context_);
        socket_ = nullptr;
        context_ = nullptr;
        is_ready = false;
        return;
    }

    is_ready = true;
    std::cout << "ZMQ publisher bound to " << bind_address << std::endl;
}

std::string ZmqPublisher::serialize_to_json(const PositionStruct& p)
{
    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{"
         << "\"subject_name\":\"" << p.subject_name << "\","
         << "\"segment_name\":\"" << p.segment_name << "\","
         << "\"translation\":[" 
         << p.translation[0] << "," 
         << p.translation[1] << "," 
         << p.translation[2] << "],"
         << "\"rotation\":[" 
         << p.rotation[0] << "," 
         << p.rotation[1] << "," 
         << p.rotation[2] << "," 
         << p.rotation[3] << "],"
         << "\"translation_type\":\"" << p.translation_type << "\","
         << "\"frame_number\":" << p.frame_number
         << "}";
    return json.str();
}

void ZmqPublisher::publish(const PositionStruct& p, const std::string& topic_name)
{
    if (!is_ready || !socket_) {
        return;
    }

    // Create a multi-part message: [topic, data]
    std::string json_data = serialize_to_json(p);
    
    // Send topic name first
    zmq_msg_t topic_msg;
    zmq_msg_init_size(&topic_msg, topic_name.size());
    memcpy(zmq_msg_data(&topic_msg), topic_name.c_str(), topic_name.size());
    int rc = zmq_msg_send(&topic_msg, socket_, ZMQ_SNDMORE);
    zmq_msg_close(&topic_msg);
    
    if (rc == -1) {
        std::cerr << "Failed to send topic: " << zmq_strerror(zmq_errno()) << std::endl;
        return;
    }
    
    // Send JSON data
    zmq_msg_t data_msg;
    zmq_msg_init_size(&data_msg, json_data.size());
    memcpy(zmq_msg_data(&data_msg), json_data.c_str(), json_data.size());
    rc = zmq_msg_send(&data_msg, socket_, ZMQ_DONTWAIT);
    zmq_msg_close(&data_msg);
    
    if (rc == -1 && zmq_errno() != EAGAIN) {
        std::cerr << "Failed to publish message: " << zmq_strerror(zmq_errno()) << std::endl;
    }
}

ZmqPublisher::~ZmqPublisher()
{
    if (socket_) {
        zmq_close(socket_);
        socket_ = nullptr;
    }
    if (context_) {
        zmq_ctx_destroy(context_);
        context_ = nullptr;
    }
}

