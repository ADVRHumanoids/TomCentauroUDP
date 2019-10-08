/*
    Simple udp master
*/
#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>
#include <unistd.h>
#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <TomCentauroUDP/packet/packet.hpp>
 
#define RECEIVER "127.0.0.1"
#define BUFLEN sizeof(TomCentauroUDP::packet::ToM2Teleopman)  //Max length of buffer

#define BUFLEN_SLAVE_2_MASTER sizeof(TomCentauroUDP::packet::Teleopman2ToM)

#define PORT_OTHER 5000   //The port on which to listen for incoming data
#define PORT_ME 5001   //The port on which to listen for incoming data
 
void die(char *s)
{
    perror(s);
    exit(1);
}

char *get_ip_str(const struct sockaddr *sa, char *s, size_t maxlen)
{
    switch(sa->sa_family) {
        case AF_INET:
            inet_ntop(AF_INET, &(((struct sockaddr_in *)sa)->sin_addr),
                    s, maxlen);
            break;

        case AF_INET6:
            inet_ntop(AF_INET6, &(((struct sockaddr_in6 *)sa)->sin6_addr),
                    s, maxlen);
            break;

        default:
            strncpy(s, "Unknown AF", maxlen);
            return NULL;
    }

    return s;
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_sender");
    ros::NodeHandle nh;
    
    struct sockaddr_in si_other, si_me, si_recv;
    int s, i;
    uint slen = sizeof(si_other);
    struct TomCentauroUDP::packet::ToM2Teleopman *pkt = (TomCentauroUDP::packet::ToM2Teleopman *)malloc(BUFLEN);
    
    // slave to master packet
    struct TomCentauroUDP::packet::Teleopman2ToM *pkt_slave_to_master =  (TomCentauroUDP::packet::Teleopman2ToM *) malloc ( BUFLEN_SLAVE_2_MASTER );
 
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
 
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT_OTHER);
     
    if (inet_aton(RECEIVER , &si_other.sin_addr) == 0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
    
    int s_recv;
    //create a UDP socket
    if ( ( s_recv=socket ( AF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) == -1 ) {
        perror ( "socket\n" );
        exit(1);
    }
    // zero out the structure
    memset ( ( char * ) &si_me, 0, sizeof ( si_me ) );

    // initialize address to bind
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons ( PORT_ME );
    si_me.sin_addr.s_addr = inet_addr ( RECEIVER );

    //bind socket to port
    if ( bind ( s_recv , ( struct sockaddr* ) &si_me, sizeof ( si_me ) ) == -1 ) {
        perror ( "bind\n" );
    }
    
    int seq_id = 0;
    auto on_pose_ref_recv = [&](const geometry_msgs::PoseStampedConstPtr& msg)
    {
        
        pkt->ref_position_ee[0] = msg->pose.position.x;
        pkt->ref_position_ee[1] = msg->pose.position.y;
        pkt->ref_position_ee[2] = msg->pose.position.z;
        
        pkt->ref_quaternion_ee[0] = msg->pose.orientation.x;
        pkt->ref_quaternion_ee[1] = msg->pose.orientation.y;
        pkt->ref_quaternion_ee[2] = msg->pose.orientation.z;
        pkt->ref_quaternion_ee[3] = msg->pose.orientation.w;
        
        pkt->packet_id = seq_id;
        
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        pkt->timestamp_master = ts.tv_sec + ts.tv_nsec*1e-9;
        
        //send the message
        if (sendto(s, pkt, BUFLEN , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }
        
        std::cout << seq_id << std::endl;
    
        
        seq_id++;
    };
    
    auto sub = nh.subscribe<geometry_msgs::PoseStamped>("/cartesian/teleop_link5/reference", 1, on_pose_ref_recv);
    
    char t[1000];
    std::cout << "OTHER: " << get_ip_str(( struct sockaddr * ) &si_other, t, 1000) << ":" << ntohs( si_other.sin_port) << std::endl;
    std::cout << "ME: " << get_ip_str(( struct sockaddr * ) &si_me, t, 1000) << ":" << ntohs( si_me.sin_port) << std::endl;
 
    while(ros::ok()) {
        
//         std::cout << si_recv.sin_addr.s_addr << " " << si_recv.sin_port << std::endl;
//         
//         int recv_len = 0;
//         if ( ( recv_len = recvfrom ( s_recv, pkt_slave_to_master, BUFLEN_SLAVE_2_MASTER, 0, ( struct sockaddr * ) &si_recv, &slen ) ) == -1 ) {
//                perror ( "recvfrom()\n" );
//             }
        
        ros::spinOnce();
        
    }
    close(s);
    return 0;
}