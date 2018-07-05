/*
    Simple udp slave
*/
#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include<sys/socket.h>
#include <unistd.h>

#include <sched.h>

#include <iit/ecat/utils.h>

#include <TomCentauroUDP/pipes.h>

#include <TomCentauroUDP/packet/packet.hpp>

// #define SENDER "192.168.0.215"
#define SENDER "192.168.0.10"
#define RECEIVER "192.168.0.2"
#define BUFLEN_PACKET sizeof(TomCentauroUDP::packet::master2slave) 
#define BUFLEN_SLAVE_2_MASTER sizeof(TomCentauroUDP::packet::slave2master)
#define PORT_MASTER_2_SLAVE 16000   //The port on which to listen for incoming data
#define PORT_SLAVE_2_MASTER 16001   //The port on which to listen for incoming data

void die(char *s)
{
    perror(s);
    exit(1);
}
 
int main(void)
{
    // set the CPU id
    
    // UDP related stuffs
    struct sockaddr_in si_me, si_other, si_recv;
    int s, s_send, i , recv_len;
    uint slen = sizeof(si_other);
    
    // master to slave packet
    struct TomCentauroUDP::packet::master2slave *pkt = (TomCentauroUDP::packet::master2slave *)malloc(BUFLEN_MASTER_2_SLAVE);
    
    // slave to master packet
    struct TomCentauroUDP::packet::slave2master *pkt_to_send = (TomCentauroUDP::packet::slave2master *)malloc(BUFLEN_SLAVE_2_MASTER);
    
    // exoskeleton pipe
    int exoskeleton_fd = open((pipe_prefix+std::string("exoskeleton_pipe")).c_str(), O_WRONLY);
    if( exoskeleton_fd < 0 ){
        die("Open exoskeleton_fd");
    }
    printf("fd : %d\n", exoskeleton_fd);
    fflush(stdout);
    
    // robot pipe
    int robot_fd = open((pipe_prefix+std::string("robot_pipe")).c_str(), O_RDONLY);
    if( robot_fd < 0 ){
        die("Open robot_fd");
    }
    printf("fd : %d\n", robot_fd);
    fflush(stdout);
     
    //create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
    
    //create send UDP socket
    if ((s_send=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
     
    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));
    
    // initialize address to bind
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT_MASTER_2_SLAVE);
    si_me.sin_addr.s_addr = inet_addr(RECEIVER);
     
    //bind socket to port
    if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        die("bind");
    }
    
    // initialize master address
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT_SLAVE_2_MASTER);
    if (inet_aton(SENDER , &si_other.sin_addr) == 0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
        
     
    //keep listening for data
    while(1)
    {
        printf("Waiting for data...");
        fflush(stdout);
         
        //try to receive some data, this is a blocking call
        if ((recv_len = recvfrom(s, pkt, BUFLEN_MASTER_2_SLAVE, 0, (struct sockaddr *) &si_recv, &slen)) == -1)
        {
            die("recvfrom()");
        }
        
        
        printf("timer master: %f - timer slave: %f \n", pkt->timer_master, (iit::ecat::get_time_ns() / 1e9));
        
        // printf test
//         printf("l_handle_trigger: %f\n" , pkt->l_handle_trigger);
//         printf("l_position_x: %f\n" , pkt->l_position_x);
//         printf("l_position_y: %f\n" , pkt->l_position_y);
//         printf("l_position_z: %f\n" , pkt->l_position_z);
//         printf("l_velocity_x: %f\n" , pkt->l_velocity_x);
//         printf("l_velocity_y: %f\n" , pkt->l_velocity_y);
//         printf("l_velocity_z: %f\n" , pkt->l_velocity_z);
//         
//         printf("r_handle_trigger: %f\n" , pkt->r_handle_trigger);
//         printf("r_position_x: %f\n" , pkt->r_position_x);
//         printf("r_position_y: %f\n" , pkt->r_position_y);
//         printf("r_position_z: %f\n" , pkt->r_position_z);
//         printf("r_velocity_x: %f\n" , pkt->r_velocity_x);
//         printf("r_velocity_y: %f\n" , pkt->r_velocity_y);
//         printf("r_velocity_z: %f\n" , pkt->r_velocity_z);
        
        // write on exoskeleton_pipe
        int bytes = write(exoskeleton_fd, (void *)pkt, BUFLEN_MASTER_2_SLAVE);
        
        // read from robot pipe
        bytes = read(robot_fd, (void *)pkt_to_send, BUFLEN_SLAVE_2_MASTER);
        
        printf("force sent: %f %f %f \n" , pkt_to_send->l_force_x, pkt_to_send->l_force_y, pkt_to_send->l_force_z);
        // put back the master timer
        pkt_to_send->timer_slave = pkt->timer_master;
        //send the message
        if (sendto(s_send, pkt_to_send, BUFLEN_SLAVE_2_MASTER , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }
//         usleep(1000); // 1 ms
    }
 
    close(s);
    close(s_send);
    close(robot_fd);
    close(exoskeleton_fd);
    return 0;
}
