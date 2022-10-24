#include "udp_server.h"
#include "tom_centauro_udp/packet/packet.hpp"
#include "tom_centauro_udp/packet/packet_ros_utils.hpp"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_client_ros");
    ros::NodeHandle nh;

    std::string addr = "127.0.0.1";
    int port = 8081;

    UdpClient cli;
    if(!cli.init(addr, port))
    {
        exit(1);
    }

    std::string ee_id = "arm2_8";
    ros::Subscriber ee_sub = nh.subscribe<geometry_msgs::PoseStamped>(
                "input_topic", 1,
                [&](const auto& msg)
    {
         tom_centauro_udp::packet::master2slave packet_to_robot;

         tom_centauro_udp::fill_pkt_ee_id(packet_to_robot, ee_id);

         packet_to_robot.run = true;

         tom_centauro_udp::fill_pkt_with_pose(packet_to_robot,
                                              *msg);

         cli.send(reinterpret_cast<uint8_t*>(&packet_to_robot),
                  sizeof(packet_to_robot));
    });

    ros::spin();
}
