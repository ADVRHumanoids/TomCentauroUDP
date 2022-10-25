#include "udp_server.h"
#include "tom_centauro_udp/packet/packet.hpp"
#include "tom_centauro_udp/packet/packet_ros_utils.hpp"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_server_ros");
    ros::NodeHandle nh;

    // bind on all interfaces, on given port
    std::string addr = "0.0.0.0";
    int port = 8081;

    UdpServer srv;
    if(!srv.bind(addr, port))
    {
        exit(1);
    }

    // ros-stuff to connect to ik
    std::map<std::string, ros::Publisher> ee_pub_map;
    std::map<std::string, ros::Subscriber> ee_sub_map;
    std::map<std::string, geometry_msgs::PoseStamped> ee_ref_map;

    // packets declaration
    tom_centauro_udp::packet::master2slave packet_to_robot;
    tom_centauro_udp::packet::slave2master packet_to_teleop;

    int nmsgs = 0;
    int nrepl = 0;

    for(;;)
    {
        // periodically print statistics
        ROS_INFO_THROTTLE(1.0, "recv %d  repl %d (client at %s)",
                          nmsgs, nrepl, srv.get_last_client_address().c_str());

        // blocking receive
        int nbytes = srv.receive(reinterpret_cast<uint8_t*>(&packet_to_robot),
                                 sizeof(packet_to_robot));

        // error handling
        if(nbytes != sizeof(packet_to_robot))
        {
            ROS_WARN("received %d != %d bytes", nbytes, sizeof(packet_to_robot));
            continue;
        }

        nmsgs++;

        // get end effector name
        std::string ee_id = packet_to_robot.ee_id;

        // retrieve or create publisher and subscriber
        auto pub_it = ee_pub_map.find(ee_id);

        ros::Publisher pub;

        if(pub_it == ee_pub_map.end())
        {
            ROS_INFO("creating publisher for ee '%s'", ee_id.c_str());

            pub = nh.advertise<geometry_msgs::PoseStamped>(
                        "cartesian/" + ee_id + "/reference", 1);

            ee_pub_map[ee_id] = pub;

            auto sub = nh.subscribe<geometry_msgs::PoseStamped>(
                        "cartesian/" + ee_id + "/current_reference", 1,
                        [ee_id, &ee_ref_map](const auto& msg)
            {
                ee_ref_map[ee_id] = *msg;
            });

            ee_sub_map[ee_id] = sub;
        }
        else
        {
            pub = pub_it->second;
        }

        // spin ros callbacks to get current reference
        ros::spinOnce();

        // reply to client with current reference if we got one
        auto ref_it = ee_ref_map.find(ee_id);

        if(ref_it != ee_ref_map.end())
        {
            tom_centauro_udp::fill_pkt_ee_id(packet_to_teleop, ee_id);

            tom_centauro_udp::fill_pkt_with_pose(packet_to_teleop,
                                                 ref_it->second);

            srv.reply(reinterpret_cast<uint8_t*>(&packet_to_teleop),
                      sizeof(packet_to_teleop));

            nrepl++;
        }

        // should we pub this reference?
        if(packet_to_robot.run)
        {
            geometry_msgs::PoseStamped msg;
            msg.header.stamp = ros::Time::now();
            tom_centauro_udp::get_pose_from_pkt(packet_to_robot,
                                                msg);
            pub.publish(msg);
        }
    }
}
