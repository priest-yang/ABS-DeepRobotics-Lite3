// DepthReceiver.hpp
#ifndef DEPTHRECEIVER_HPP
#define DEPTHRECEIVER_HPP

#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <boost/asio.hpp>

using boost::asio::ip::udp;
using namespace Eigen;

class DepthReceiver {
public:
    DepthReceiver();
    void start();
    void stop();
    Eigen::VectorXf get_latest_data();
    ~DepthReceiver();

private:
    void receive_depth_data();

    const int PORT = 12125;                // Port to receive data
    int OUT_DIMEN_ = 11;              // Define the output dimension
    std::thread receiver_thread;
    std::atomic<bool> is_running;
    std::mutex data_mutex;
    Eigen::VectorXf latest_data;
};

#endif // DEPTHRECEIVER_HPP
