#include "tom_centauro_udp/packet/packet.hpp"
#include "tom_centauro_udp/packet/packet_ros_utils.hpp"

#include "tom_centauro_udp/communication/UDPSocket.h"

#include <ros/ros.h>
#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "udp_dummy_ros");
    ros::NodeHandle nh;

    // udp non blocking socket
    UDPSocket<tom_centauro_udp::packet::master2slave, tom_centauro_udp::packet::slave2master> udp_socket;

    // ToM address (change it to your local ip)
    const std::string local_address = "192.168.10.101";
    const int local_port = 8080;

    // Centauro address
    const std::string remote_address = "192.168.10.101";
    const int remote_port = 8081;

    udp_socket.initialize();
    udp_socket.bindLocal(local_address.c_str(), local_port);
    udp_socket.connectRemote(remote_address.c_str(), remote_port);

    // reference message from ToM to Centauro
    tom_centauro_udp::packet::master2slave packet_to_robot;
    // state message from Centauro to ToM
    tom_centauro_udp::packet::slave2master packet_to_tom;

    // end effector frame to be controlled
    const std::string ee_id = "arm2_8";

    tom_centauro_udp::fill_pkt_ee_id(packet_to_robot, ee_id);
    packet_to_robot.rotation[0] = 1;

    packet_to_robot.run = false;

    ros::Subscriber ee_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/cartesian/arm2_8/reference", 1,
        [&](const auto &msg)
        {
            ROS_INFO_THROTTLE(1, "cartesian/arm2_8/reference");
        });

    auto tx_callback = [&]()
    {
        // thead callback to send reference
        auto next = std::chrono::high_resolution_clock::now();
        while (ros::ok())
        {
            // packet_to_robot.position_z = sin(next.time_since_epoch().count());
            udp_socket.socketSend(packet_to_robot);

            ros::spinOnce();
            next = next + std::chrono::milliseconds(10);
            std::this_thread::sleep_until(next);
        }
    };

    double t = 0;
    const double dt = 0.001;
    const double period = 3.0;
    const double amplitude = 0.01;

    auto rx_callback = [&]()
    {
        // non blocking receive thead callback to receive current robot state
        auto next = std::chrono::high_resolution_clock::now();
        while (ros::ok())
        {
            int ret = udp_socket.socketReceive(packet_to_tom, sizeof(packet_to_tom));
            if (ret >= sizeof(packet_to_tom))
            {
                packet_to_robot.run = true;
                packet_to_robot.position_z = packet_to_tom.position_z + amplitude * std::sin(2.0 * M_PI * t / period);
                packet_to_robot.position_x = packet_to_tom.position_x;
                packet_to_robot.position_y = packet_to_tom.position_y + amplitude * std::cos(2.0 * M_PI * t / period);
            }
            else
            {

                // packet_to_robot.run =false;
            }
            // std::cout << packet_to_tom.position_x << std::endl;
            t = t + dt;
            next = next + std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next);
        }
    };

    std::thread tx_thread(tx_callback);
    std::thread rx_thread(rx_callback);
    tx_thread.join();
    rx_thread.join();

    return 0;
}
