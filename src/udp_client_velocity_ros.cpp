#include "udp_server.h"
#include "tom_centauro_udp/packet/packet.hpp"
#include "tom_centauro_udp/packet/packet_ros_utils.hpp"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tom_centauro_udp_client");
    ros::NodeHandle n("~");

    std::string addr = n.param<std::string>("addr", "127.0.0.1");
    int port = n.param("port", 8081);

    std::string ee_id = n.param<std::string>("ee_id", "arm2_8");

    double v_x = n.param("v_x", 0.0);
    double v_y = n.param("v_y", 0.0);
    double v_yaw = n.param("v_yaw", 0.0);

    UdpClient<tom_centauro_udp::packet::slave2master,
              tom_centauro_udp::packet::master2slave> cli;

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
        ROS_INFO_THROTTLE(1, "waiting for server to reply..");

        cli.try_send(packet_to_robot);

        packet_from_robot.magic_code = 0;
        if(cli.try_receive(packet_from_robot))
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

    // set velocity control
    packet_to_robot.velocity_ctrl = true;

    double t = 0;
    double dt = 0.001;
    int nrepl = 0;

    for(;;)
    {
        packet_to_robot.vel_xy[0] = v_x;
        packet_to_robot.vel_xy[1] = v_y;
        packet_to_robot.vel_yaw = v_yaw;

        // send command to slave
        cli.try_send(packet_to_robot);

        // try receive from robot
        packet_from_robot.magic_code = 0;
        if(cli.try_receive(packet_from_robot))
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
