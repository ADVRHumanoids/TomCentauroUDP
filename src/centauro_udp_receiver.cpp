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

#define ADDRESS_OTHER "192.168.0.2"
#define ADDRESS_ME "192.168.0.81"
#define BUFLEN_MASTER_2_SLAVE sizeof(TomCentauroUDP::packet::master2slave) 
#define BUFLEN_SLAVE_2_MASTER sizeof(TomCentauroUDP::packet::slave2master)
#define PORT_ME 5001   //The port on which to listen for incoming data
#define PORT_OTHER 5001   //The port on which to listen for incoming data

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
    int s_recv, s_send, i , recv_len;
    uint slen = sizeof(si_other);
    
    fd_set cset;
    struct timeval tval;
    tval.tv_sec = 0; tval.tv_usec = 2000; //2ms
    //keep listening for data
    
    // master to slave packet
    struct TomCentauroUDP::packet::master2slave *pkt_to_receive = (TomCentauroUDP::packet::master2slave *)malloc(BUFLEN_MASTER_2_SLAVE);
    
    
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
    if ((s_recv=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
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
    si_me.sin_port = htons(PORT_ME);
    si_me.sin_addr.s_addr = inet_addr(ADDRESS_ME);
     
    //bind socket to port
    if( bind(s_recv , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        die("bind");
    }
    
    // initialize master address
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT_OTHER);
    if (inet_aton(ADDRESS_OTHER , &si_other.sin_addr) == 0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
        
     
    //keep listening for data
    while(1)
    {
        printf("Waiting for data...\n");
        fflush(stdout);
	
	// read from robot pipe
        int bytes = read(robot_fd, (void *)pkt_to_send, BUFLEN_SLAVE_2_MASTER);
        
	
	printf("timer master: %f - timer slave: %f \n", pkt_to_receive->timer_master, (iit::ecat::get_time_ns() / 1e9));
	
	pkt_to_send->timer_master = (iit::ecat::get_time_ns() / 1e9);
	
	// put back the master timer
        pkt_to_send->timer_slave = pkt_to_receive->timer_master;
        //send the message
        if (sendto(s_send, pkt_to_send, BUFLEN_SLAVE_2_MASTER , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }
         
	FD_ZERO(&cset);
	FD_SET(s_recv, &cset);
	int n = select(FD_SETSIZE, &cset, NULL, NULL, &tval);
	if (n == -1)
	{
	    die("select() failed");
	}
	if (FD_ISSET(s_recv, &cset))
	{
	  //try to receive some data, this is a blocking call
	  if ((recv_len = recvfrom(s_recv, pkt_to_receive, BUFLEN_MASTER_2_SLAVE, 0, (struct sockaddr *) &si_recv, &slen)) == -1)
	  {
	      die("recvfrom()");
	  }
	}
        

        // write on exoskeleton_pipe
        bytes = write(exoskeleton_fd, (void *)pkt_to_receive, BUFLEN_MASTER_2_SLAVE);
       
//         usleep(1000); // 1 ms
    }
 
    close(s_recv);
    close(s_send);
    close(robot_fd);
    close(exoskeleton_fd);
    return 0;
}
