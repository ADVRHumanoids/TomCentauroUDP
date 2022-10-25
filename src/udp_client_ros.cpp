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

    std::string ee_id = "arm2_8";

    if(argc > 1)
    {
        ee_id = argv[1];
    }

    UdpClient cli;
    if(!cli.init(addr, port))
    {
        exit(1);
    }

    bool got_msg_from_robot = false;
    int nrepl = 0;


    ros::Subscriber ee_sub = nh.subscribe<geometry_msgs::PoseStamped>(
                "input_topic", 1,
                [&](const auto& msg)
    {
        ROS_INFO_THROTTLE(1, "got ros msg from input topic");

        tom_centauro_udp::packet::master2slave packet_to_robot;

        tom_centauro_udp::fill_pkt_ee_id(packet_to_robot, ee_id);

        packet_to_robot.run = got_msg_from_robot;

        tom_centauro_udp::fill_pkt_with_pose(packet_to_robot,
                                             *msg);

        cli.send(reinterpret_cast<uint8_t*>(&packet_to_robot),
                 sizeof(packet_to_robot));

        tom_centauro_udp::packet::slave2master packet_from_robot;

        if(cli.try_receive(reinterpret_cast<uint8_t*>(&packet_from_robot),
                           sizeof(packet_from_robot)))
        {
            got_msg_from_robot = true;
            nrepl++;

            geometry_msgs::PoseStamped pose;
            tom_centauro_udp::get_pose_from_pkt(packet_from_robot, pose);

            ROS_INFO_THROTTLE(1, "got %d replies from robot", nrepl);
            ROS_INFO_STREAM_THROTTLE(1, "" << pose);
        }
    });

    ros::spin();
}
