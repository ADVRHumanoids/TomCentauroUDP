#ifndef PACKET_ROS_UTILS_HPP
#define PACKET_ROS_UTILS_HPP

#include "packet.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

namespace tom_centauro_udp
{

bool check_pkt_valid(const packet::master2slave& pkt)
{
    return pkt.magic_code == pkt.expected_magic_code;
}

bool check_pkt_valid(const packet::slave2master& pkt)
{
    return pkt.magic_code == pkt.expected_magic_code;
}

template <typename packet_t>
void fill_pkt_ee_id(packet_t& pkt,
                    const std::string& ee_id)
{
    if(ee_id.length() >= sizeof(pkt.ee_id))
    {
        throw std::runtime_error("ee_id too long: " + ee_id);
    }

    strncpy(pkt.ee_id, ee_id.c_str(), sizeof(pkt.ee_id));
    pkt.ee_id[ee_id.length()] = '\0';

}

template <typename packet_t>
void fill_pkt_with_pose(packet_t& pkt,
                        const geometry_msgs::PoseStamped& pose)
{
    pkt.position_x = pose.pose.position.x;
    pkt.position_y = pose.pose.position.y;
    pkt.position_z = pose.pose.position.z;

    Eigen::Quaternionf q(pose.pose.orientation.w,
                         pose.pose.orientation.x,
                         pose.pose.orientation.y,
                         pose.pose.orientation.z);

    q.normalize();

    // Eigen::Matrix3f::Map(pkt.rotation) = q.toRotationMatrix();

    pkt.rotation[0] = q.w();
    pkt.rotation[1] = q.x();
    pkt.rotation[2] = q.y();
    pkt.rotation[3] = q.z();

}

template <typename packet_t>
void get_pose_from_pkt(packet_t& pkt,
                       geometry_msgs::PoseStamped& pose)
{
    pose.pose.position.x = pkt.position_x;
    pose.pose.position.y = pkt.position_y;
    pose.pose.position.z = pkt.position_z;

    // Eigen::Quaternionf q(Eigen::Matrix3f::Map(pkt.rotation));

    Eigen::Quaternionf q(pkt.rotation[0],
                         pkt.rotation[1],
                         pkt.rotation[2],
                         pkt.rotation[3]);

    q.normalize();

    pose.pose.orientation.x = q.w();
    pose.pose.orientation.y = q.x();
    pose.pose.orientation.z = q.y();
    pose.pose.orientation.w = q.z();
}

}

#endif // PACKET_ROS_UTILS_HPP
