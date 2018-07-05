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

#include <CentauroUDP/packet/master2slave.h>
 
#define RECEIVER "192.168.0.2"
#define BUFLEN sizeof(CentauroUDP::packet::master2slave)  //Max length of buffer
#define PORT 16000   //The port on which to send data
 
void die(char *s)
{
    perror(s);
    exit(1);
}
 
int main(void)
{
    struct sockaddr_in si_other;
    int s, i;
    uint slen = sizeof(si_other);
    struct CentauroUDP::packet::master2slave *pkt = (CentauroUDP::packet::master2slave *)malloc(BUFLEN);
 
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
 
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
     
    if (inet_aton(RECEIVER , &si_other.sin_addr) == 0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
    
    // test initialization
    pkt->l_position_x = 0.255;
    pkt->l_position_y = 0.355;
    pkt->l_position_z = -1.555;
 
    while(1)
    {
         pkt->l_position_x += 0.001;
         pkt->l_position_y -= 0.001;
         pkt->l_position_z += 0.001;
        //send the message
        if (sendto(s, pkt, BUFLEN , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }
        usleep(10000); // 10 ms
        
    }
 
    close(s);
    return 0;
}
