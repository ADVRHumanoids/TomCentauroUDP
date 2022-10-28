#include "udp_server.h"
#include "tom_centauro_udp/packet/packet.hpp"
#include "tom_centauro_udp/packet/packet_ros_utils.hpp"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tom_centauro_udp_client");
    ros::NodeHandle n;

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

    tom_centauro_udp::packet::master2slave packet_to_robot;
    tom_centauro_udp::fill_pkt_ee_id(packet_to_robot, ee_id);
    packet_to_robot.run = false;

    tom_centauro_udp::packet::slave2master packet_from_robot;

    while(!got_msg_from_robot)
    {
        cli.try_send(reinterpret_cast<uint8_t*>(&packet_to_robot),
                     sizeof(packet_to_robot));

        packet_from_robot.magic_code = 0;
        if(cli.try_receive(reinterpret_cast<uint8_t*>(&packet_from_robot),
                           sizeof(packet_from_robot)) > 0)
        {
            if(!tom_centauro_udp::check_pkt_valid(packet_from_robot))
            {
                ROS_ERROR("invalid packet received");
                continue;
            }

            if(std::string(packet_from_robot.ee_id) != ee_id)
            {
                ROS_WARN("packet for ee %s != %s received",
                         packet_from_robot.ee_id, ee_id.c_str());
                continue;
            }

            got_msg_from_robot = true;
        }

        usleep(100000);
    }

    // set run to true
    packet_to_robot.run = true;

    geometry_msgs::PoseStamped initial_pose;
    tom_centauro_udp::get_pose_from_pkt(packet_from_robot, initial_pose);
    std::cout << "initial pose for " << packet_from_robot.ee_id << " received: \n" << initial_pose << "\n";

    double t = 0;
    const double dt = 0.001;
    const double period = 3.0;
    const double amplitude = 0.1;
    int nrepl = 0;

    for(;;)
    {
        // fill pose to send
        geometry_msgs::PoseStamped pose_to_send = initial_pose;
        pose_to_send.pose.position.z += amplitude*std::sin(2*M_PI*t/period);

        tom_centauro_udp::fill_pkt_with_pose(packet_to_robot,
                                             pose_to_send);

        // send command to slave
        cli.try_send(reinterpret_cast<uint8_t*>(&packet_to_robot),
                     sizeof(packet_to_robot));

        // try receive from robot
        packet_from_robot.magic_code = 0;
        if(cli.try_receive(reinterpret_cast<uint8_t*>(&packet_from_robot),
                           sizeof(packet_from_robot)) > 0)
        {
            if(!tom_centauro_udp::check_pkt_valid(packet_from_robot))
            {
                ROS_ERROR("invalid packet received");
                continue;
            }

            if(std::string(packet_from_robot.ee_id) != ee_id)
            {
                ROS_WARN("packet for ee %s != %s received",
                         packet_from_robot.ee_id, ee_id.c_str());
                continue;
            }

            nrepl++;

            ROS_INFO_THROTTLE(1, "got %d replies from robot", nrepl);
        }

        t += dt;
        usleep(dt*1e6);
    }
}
