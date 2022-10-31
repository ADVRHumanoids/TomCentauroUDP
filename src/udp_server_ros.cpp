#include "udp_server.h"
#include "tom_centauro_udp/packet/packet.hpp"
#include "tom_centauro_udp/packet/packet_ros_utils.hpp"

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_server_ros");
    ros::NodeHandle nh, nhp("~");

    // params
    const double max_vel_xy = nhp.param("max_vel_xy", 0.20);
    const double max_vel_yaw = nhp.param("max_vel_yaw", 0.20);

    // bind on all interfaces, on given port
    std::string addr = "0.0.0.0";
    int port = nhp.param("port", 8081);

    UdpServer<tom_centauro_udp::packet::master2slave,
            tom_centauro_udp::packet::slave2master> srv;
    if(!srv.bind(addr, port))
    {
        exit(1);
    }

    // ros-stuff to connect to ik
    std::map<std::string, ros::Publisher> ee_pub_map;
    std::map<std::string, ros::Publisher> ee_vel_pub_map;
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
        ROS_INFO_THROTTLE(2.0, "recv %d  repl %d (client at %s)",
                          nmsgs, nrepl, srv.get_last_client_address().c_str());

        // blocking receive
        if(!srv.receive(packet_to_robot))
        {
            ROS_WARN("failed to receive packet to robot");
            continue;
        }

        // safety checks
        if(!tom_centauro_udp::check_pkt_valid(packet_to_robot))
        {
            ROS_ERROR("invalid packet received");
            continue;
        }

        nmsgs++;

        // get end effector name
        std::string ee_id = packet_to_robot.ee_id;

        // retrieve or create publisher and subscriber
        ros::Publisher pub;

        // velocity mode
        if(packet_to_robot.velocity_ctrl)
        {
            auto pub_it = ee_vel_pub_map.find(ee_id);

            if(pub_it == ee_vel_pub_map.end())
            {
                ROS_INFO("creating velocity publisher for ee '%s'", ee_id.c_str());

                pub = nh.advertise<geometry_msgs::TwistStamped>(
                            "cartesian/" + ee_id + "/velocity_reference", 1);

                ee_vel_pub_map[ee_id] = pub;

            }
            else
            {
                pub = pub_it->second;
            }
        }
        // position mode
        else
        {
            auto pub_it = ee_pub_map.find(ee_id);

            if(pub_it == ee_pub_map.end())
            {
                ROS_INFO("creating publisher for ee '%s'", ee_id.c_str());

                pub = nh.advertise<geometry_msgs::PoseStamped>(
                            "cartesian/" + ee_id + "/reference", 1);

                ee_pub_map[ee_id] = pub;

            }
            else
            {
                pub = pub_it->second;
            }
        }

        if(ee_sub_map.count(ee_id) == 0)
        {
            // current reference subscriber
            auto sub = nh.subscribe<geometry_msgs::PoseStamped>(
                        "cartesian/" + ee_id + "/current_reference", 1,
                        [ee_id, &ee_ref_map](const auto& msg)
            {
                ee_ref_map[ee_id] = *msg;
            });

            ee_sub_map[ee_id] = sub;
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

            srv.reply(packet_to_teleop);

            nrepl++;
        }

        // should we pub this reference?
        if(packet_to_robot.run)
        {
            if(packet_to_robot.velocity_ctrl)
            {
                geometry_msgs::TwistStamped msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = ee_id;  // note: local control
                msg.twist.linear.x = packet_to_robot.vel_xy[0] * max_vel_xy;
                msg.twist.linear.y = packet_to_robot.vel_xy[1] * max_vel_xy;
                msg.twist.angular.z = packet_to_robot.vel_yaw * max_vel_yaw;
                pub.publish(msg);
            }
            else
            {
                geometry_msgs::PoseStamped msg;
                msg.header.stamp = ros::Time::now();
                tom_centauro_udp::get_pose_from_pkt(packet_to_robot,
                                                    msg);
                pub.publish(msg);
            }
        }
    }
}
