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

#include <TomCentauroUDP/packet/packet.hpp>

#include <XBotInterface/Utils.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

#define ADDRESS_OTHER "192.168.0.2"
#define ADDRESS_ME "192.168.0.81"
#define BUFLEN_MASTER_2_SLAVE sizeof(TomCentauroUDP::packet::master2slave)
#define BUFLEN_SLAVE_2_MASTER sizeof(TomCentauroUDP::packet::slave2master)
#define PORT_ME 5001   //The port on which to listen for incoming data
#define PORT_OTHER 5001   //The port on which to listen for incoming data

int main ( void ) {
    // set the logger verbosity
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::LOW);

    // UDP related stuffs
    struct sockaddr_in si_me, si_other, si_recv;
    int s_recv, s_send, i , recv_len;
    uint slen = sizeof ( si_other );

    // select config
    fd_set cset;
    struct timeval tval;
    tval.tv_sec = 0;
    tval.tv_usec = 2000; //2ms
    //keep listening for data

    // master to slave packet
    struct TomCentauroUDP::packet::master2slave *pkt_to_receive = ( TomCentauroUDP::packet::master2slave * ) malloc ( BUFLEN_MASTER_2_SLAVE );
    // slave to master packet
    struct TomCentauroUDP::packet::slave2master *pkt_to_send = ( TomCentauroUDP::packet::slave2master * ) malloc ( BUFLEN_SLAVE_2_MASTER );

    // exoskeleton pipe
    XBot::PublisherNRT<TomCentauroUDP::packet::master2slave> exoskeleton_pub("exoskeleton_pipe");
    // robot pipe
    XBot::SubscriberNRT<TomCentauroUDP::packet::slave2master> robot_sub("robot_pipe");

    //create a UDP socket
    if ( ( s_recv=socket ( AF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) == -1 ) {
        XBot::Logger::error ( "socket\n" );
        exit(1);
    }
    
//     TBD check if needed
//     struct timeval read_timeout;
//     read_timeout.tv_sec = 0;
//     read_timeout.tv_usec = 10;
//     setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

    //create send UDP socket
    if ( ( s_send=socket ( AF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) == -1 ) {
        XBot::Logger::error ( "socket\n" );
        exit(1);
    }

    // zero out the structure
    memset ( ( char * ) &si_me, 0, sizeof ( si_me ) );

    // initialize address to bind
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons ( PORT_ME );
    si_me.sin_addr.s_addr = inet_addr ( ADDRESS_ME );

    //bind socket to port
    if ( bind ( s_recv , ( struct sockaddr* ) &si_me, sizeof ( si_me ) ) == -1 ) {
        XBot::Logger::error ( "bind\n" );
    }

    // initialize master address
    memset ( ( char * ) &si_other, 0, sizeof ( si_other ) );
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons ( PORT_OTHER );
    if ( inet_aton ( ADDRESS_OTHER , &si_other.sin_addr ) == 0 ) {
        XBot::Logger::error ( "inet_aton() failed\n" );
        exit (1);
    }


    int count = 0;
    int retry = 0;
    
    //keep listening for data
    while ( 1 ) {
        XBot::Logger::info ( "Waiting for data...\n" );
        
        // read from robot pipe
        robot_sub.read(*pkt_to_send);
//         int bytes = read ( robot_fd, ( void * ) pkt_to_send, BUFLEN_SLAVE_2_MASTER );

        pkt_to_send->timer_master = count;

        // put back the master timer
        pkt_to_send->timer_slave = pkt_to_receive->timer_master;
        //send the message
        if ( sendto ( s_send, pkt_to_send, BUFLEN_SLAVE_2_MASTER , 0 , ( struct sockaddr * ) &si_other, slen ) ==-1 ) {
            XBot::Logger::error ( "sendto()\n" );
            exit(1);
        }

        FD_ZERO ( &cset );
        FD_SET ( s_recv, &cset );
        int n = select ( FD_SETSIZE, &cset, NULL, NULL, &tval );
        if ( n == -1 ) {
            XBot::Logger::error ( "select() failed\n" );
        }
        if ( FD_ISSET ( s_recv, &cset ) ) {
            //try to receive some data, this is a blocking call
            if ( ( recv_len = recvfrom ( s_recv, pkt_to_receive, BUFLEN_MASTER_2_SLAVE, 0, ( struct sockaddr * ) &si_recv, &slen ) ) == -1 ) {
                XBot::Logger::error ( "recvfrom()\n" );
            }
        }
            
            // TBD otherwise try to receive some data, this is a not blocking call - trying to empty the buffer
//             while ((recv_len = recvfrom(s, pkt, BUFLEN_MASTER_2_SLAVE, 0, (struct sockaddr *) &si_recv, &slen)) != -1)
//             {
//                 retry++;
//                 XBot::Logger::info("recv_len: %d - retry number: %d\n", recv_len, retry);
//             }
// 
//             retry = 0;


        // write on exoskeleton_pipe
        exoskeleton_pub.write(*pkt_to_receive);
//         bytes = write ( exoskeleton_fd, ( void * ) pkt_to_receive, BUFLEN_MASTER_2_SLAVE );

//         usleep(1000); // 1 ms
        count++;
    }

    close ( s_recv );
    close ( s_send );
    return 0;
}
