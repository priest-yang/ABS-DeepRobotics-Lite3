// DepthReceiver.cpp
#include "sensor_receiver.hpp"
#include <iostream>

DepthReceiver::DepthReceiver() : latest_data(this->OUT_DIMEN_), is_running(false) {
    latest_data.setConstant(6.00);
    std::cout << "DepthReceiver initialized." << std::endl; // Debug statement
}

void DepthReceiver::start() {
    std::cout << "Starting DepthReceiver..." << std::endl; // Debug statement
    is_running = true;
    receiver_thread = std::thread(&DepthReceiver::receive_depth_data, this);
    std::cout << "DepthReceiver started." << std::endl; // Debug statement
}

void DepthReceiver::stop() {
    is_running = false;
    if (receiver_thread.joinable()) {
        receiver_thread.join();
    }
    std::cout << "DepthReceiver stopped." << std::endl; // Debug statement
}

Eigen::VectorXf DepthReceiver::get_latest_data() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return latest_data;
}

DepthReceiver::~DepthReceiver() {
    stop();
}

void DepthReceiver::receive_depth_data() {
    boost::asio::io_service io_service;
    udp::socket socket(io_service, udp::endpoint(udp::v4(), this->PORT));
    std::vector<char> recv_buf(this->OUT_DIMEN_ * sizeof(float));
    udp::endpoint sender_endpoint;

    while (is_running) {
        size_t len = socket.receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
        if (len != this->OUT_DIMEN_ * sizeof(float)) {
            std::cerr << "Received unexpected size: " << len << std::endl;
            continue; // or handle error appropriately
        }

        Eigen::VectorXf received_vector(this->OUT_DIMEN_);
        std::memcpy(received_vector.data(), recv_buf.data(), this->OUT_DIMEN_ * sizeof(float));

        {
            std::lock_guard<std::mutex> lock(data_mutex);
            latest_data = received_vector;
        }
    }
}
