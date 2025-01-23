#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

class TreadmillROSClient {
public:
    TreadmillROSClient(const std::string& server_ip, const std::string& server_port, ros::NodeHandle& nh)
        : server_ip_(server_ip), server_port_(server_port), io_service_(), socket_(io_service_)
    {
        command_subscriber_ = nh.subscribe("/treadmill_command", 10, &TreadmillROSClient::sendCommand, this);
        status_publisher_ = nh.advertise<std_msgs::String>("/treadmill_status", 10);
    }

    void connectToServer() {
        try {
            tcp::resolver resolver(io_service_);
            tcp::resolver::query query(server_ip_, server_port_);
            tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

            boost::asio::connect(socket_, endpoint_iterator);
            ROS_INFO("Connected to treadmill server at %s:%s", server_ip_.c_str(), server_port_.c_str());
        } catch (std::exception& e) {
            ROS_ERROR("Failed to connect to treadmill server: %s", e.what());
            ros::shutdown();
        }
    }

    void sendCommand(const std_msgs::String::ConstPtr& msg) {
        try {
            ROS_INFO("Sending command to server: %s", msg->data.c_str());
            boost::asio::write(socket_, boost::asio::buffer(msg->data + "\n"));

            char response[1024];
            size_t len = socket_.read_some(boost::asio::buffer(response));
            std::string response_str(response, len);

            std_msgs::String ros_msg;
            ros_msg.data = response_str;
            status_publisher_.publish(ros_msg);

            ROS_INFO("Received response from server: %s", response_str.c_str());
        } catch (std::exception& e) {
            ROS_ERROR("Error during communication with server: %s", e.what());
        }
    }

    void closeConnection() {
        try {
            socket_.close();
            ROS_INFO("Connection to treadmill server closed.");
        } catch (std::exception& e) {
            ROS_WARN("Error while closing connection: %s", e.what());
        }
    }

private:
    std::string server_ip_;
    std::string server_port_;
    boost::asio::io_service io_service_;
    tcp::socket socket_;
    ros::Subscriber command_subscriber_;
    ros::Publisher status_publisher_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "treadmill_ros_client");
    ros::NodeHandle nh("~");

    std::string server_ip, server_port;
    nh.param<std::string>("server_ip", server_ip, "169.254.33.40");
    nh.param<std::string>("server_port", server_port, "5000");

    TreadmillROSClient client(server_ip, server_port, nh);
    client.connectToServer();

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    client.closeConnection();
    return 0;
}