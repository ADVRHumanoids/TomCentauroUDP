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

#include <ros/ros.h>
#include <centauro_tools/HeriHand.h>
#include <centauro_tools/HeriHandFingersControl.h>

#include <TomCentauroUDP/packet/packet.hpp>

#include <XBotInterface/Utils.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

//#define ADDRESS_OTHER "192.168.0.100"
//#define ADDRESS_ME "192.168.0.200"

#define ADDRESS_OTHER "10.24.8.3"
//#define ADDRESS_OTHER "10.24.8.100"
#define ADDRESS_ME "10.24.8.100"

#define BUFLEN_MASTER_2_SLAVE sizeof(TomCentauroUDP::packet::ToM2Teleopman)
#define BUFLEN_SLAVE_2_MASTER sizeof(TomCentauroUDP::packet::Teleopman2ToM)

#define PORT_ME 2001   //The port on which to listen for incoming data
#define PORT_OTHER 2002   //The port on which to listen for incoming data

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

double current_1, current_2, current_3, current_4;
double q_1, q_2, q_3, q_4;
double analogs_1_1, analogs_1_2, analogs_1_3;
double analogs_2_1, analogs_2_2, analogs_2_3;
double analogs_3_1, analogs_3_2, analogs_3_3;
double analogs_4_1, analogs_4_2, analogs_4_3;



void heriCallback (const centauro_tools::HeriHandState::ConstPtr & msg) {
    
    current_1 = msg->finger_1.current;
    current_2 = msg->finger_2.current;
    current_3 = msg->finger_3.current;
    current_4 = msg->finger_4.current;
    
    q_1 = msg->finger_1.motor_position;
    q_2 = msg->finger_2.motor_position;
    q_3 = msg->finger_3.motor_position;
    q_4 = msg->finger_4.motor_position;
    
    analogs_1_1 = msg->finger_1.analogs.analog_1;
    analogs_1_2 = msg->finger_1.analogs.analog_2;
    analogs_1_3 = msg->finger_1.analogs.analog_3;
    
    analogs_2_1 = msg->finger_2.analogs.analog_1;
    analogs_2_2 = msg->finger_2.analogs.analog_2;
    analogs_2_3 = msg->finger_2.analogs.analog_3;
    
    analogs_3_1 = msg->finger_3.analogs.analog_1;
    analogs_3_2 = msg->finger_3.analogs.analog_2;
    analogs_3_3 = msg->finger_3.analogs.analog_3;
    
    analogs_4_1 = msg->finger_4.analogs.analog_1;
    analogs_4_2 = msg->finger_4.analogs.analog_2;
    analogs_4_3 = msg->finger_4.analogs.analog_3;
    

}

void fill_heri_hand(struct TomCentauroUDP::packet::Teleopman2ToM* pkt) {
    pkt->i_fingers[0] = current_1;
    pkt->i_fingers[1] = current_2;
    pkt->i_fingers[2] = current_3;
    pkt->i_fingers[3] = current_4;
    
    pkt->q_fingers[0] = q_1;
    pkt->q_fingers[1] = q_2;
    pkt->q_fingers[2] = q_3;
    pkt->q_fingers[3] = q_4;
    
    pkt->pressure_fingers[0] = analogs_1_1;
    pkt->pressure_fingers[1] = analogs_1_2;
    pkt->pressure_fingers[2] = analogs_1_3;
    pkt->pressure_fingers[3] = analogs_2_1;
    pkt->pressure_fingers[4] = analogs_2_2;
    pkt->pressure_fingers[5] = analogs_2_3;
    pkt->pressure_fingers[6] = analogs_3_1;
    pkt->pressure_fingers[7] = analogs_3_2;
    pkt->pressure_fingers[8] = analogs_3_3;
    pkt->pressure_fingers[9] = analogs_4_1;
    pkt->pressure_fingers[10] = analogs_4_2;
    pkt->pressure_fingers[11] = analogs_4_3;

}

int main ( int argc, char* argv[] ) {
    
    ros::init(argc, argv, "udp_receiver");
    ros::NodeHandle nh;

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
    struct TomCentauroUDP::packet::ToM2Teleopman *pkt_master_to_slave =  (TomCentauroUDP::packet::ToM2Teleopman *) malloc ( BUFLEN_MASTER_2_SLAVE );
    // slave to master packet
    struct TomCentauroUDP::packet::Teleopman2ToM *pkt_slave_to_master =  (TomCentauroUDP::packet::Teleopman2ToM *) malloc ( BUFLEN_SLAVE_2_MASTER );

    // exoskeleton pipe
    XBot::PublisherNRT<TomCentauroUDP::packet::ToM2Teleopman> exoskeleton_pub("exoskeleton_pipe");
    // robot pipe
    XBot::SubscriberNRT<TomCentauroUDP::packet::Teleopman2ToM> robot_sub("robot_pipe");

    //create a UDP socket
    if ( ( s_recv=socket ( AF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) == -1 ) {
        XBot::Logger::error ( "socket\n" );
        exit(1);
    }
    
    /// set the buffer size = 2 * packet's size from master
    size_t send_buffer = BUFLEN_MASTER_2_SLAVE*2;
    if (setsockopt(s_recv, SOL_SOCKET, SO_RCVBUF, &send_buffer, sizeof(send_buffer)) == -1){
          //error
         XBot::Logger::error ( "setsockopt()\n" );
    }
    XBot::Logger::info("size: %d\n", BUFLEN_MASTER_2_SLAVE);
    
//     TBD check if needed
     struct timeval read_timeout;
     read_timeout.tv_sec = 0;
     read_timeout.tv_usec = 1000;
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
    
    char t[1000];
    std::cout << "OTHER: " << get_ip_str(( struct sockaddr * ) &si_other, t, 1000) << ":" << ntohs( si_other.sin_port) << std::endl;
    std::cout << "ME: " << get_ip_str(( struct sockaddr * ) &si_me, t, 1000) << ":" << ntohs( si_me.sin_port) << std::endl;

    ros::Subscriber sub = nh.subscribe("heri_hand_state", 1, heriCallback);
    ros::Publisher pub = nh.advertise<centauro_tools::HeriHandFingersControl>("heri_hand_fingers_control", 100);

    int count = 0;
    int retry = 0;
    
    // clear the data struct
    memset ( pkt_master_to_slave, 0, sizeof ( *pkt_master_to_slave ) );
    memset ( pkt_slave_to_master, 0, sizeof ( *pkt_slave_to_master ) );
    
    centauro_tools::HeriHandFingersControl heri_ctrl_msg;
    
    //keep listening for data
    while ( 1 ) {
        
        // read from robot pipe
        int bytes = robot_sub.read(*pkt_slave_to_master);
        
        if(bytes == 0) {
            //usleep(100000);
            //XBot::Logger::warning ( "Cannot recv from robot pipe\n" );
            continue;
        }
        
        // fill the missing fields
        pkt_slave_to_master->packet_id = count;
        // TODO timer
        
        fill_heri_hand(pkt_slave_to_master);
        
        //send the message
        if ( sendto ( s_send, pkt_slave_to_master, BUFLEN_SLAVE_2_MASTER , 0 , ( struct sockaddr * ) &si_other, slen ) == -1 ) {
            perror("sendto()");
            exit(1);
        }
        //else
            //XBot::Logger::info("sending");

        FD_ZERO ( &cset );
        FD_SET ( s_recv, &cset );
        int n = select ( FD_SETSIZE, &cset, NULL, NULL, &tval );
        if ( n == -1 ) {
            XBot::Logger::error ( "select() failed\n" );
        }
        if ( FD_ISSET ( s_recv, &cset ) ) {
            //try to receive some data, this is a blocking call
            if ( ( recv_len = recvfrom ( s_recv, pkt_master_to_slave, BUFLEN_MASTER_2_SLAVE, 0, ( struct sockaddr * ) &si_recv, &slen ) ) == -1 ) {
                XBot::Logger::error ( "recvfrom()\n" );
            }
            else {
                
                //XBot::Logger::info("receiving\n");
                // write on exoskeleton_pipe
                exoskeleton_pub.write(*pkt_master_to_slave);
                
                // command the HERI hand 
                heri_ctrl_msg.percentage_1 = pkt_master_to_slave->ref_closure_fingers[0];
                heri_ctrl_msg.percentage_2 = pkt_master_to_slave->ref_closure_fingers[1];
                heri_ctrl_msg.percentage_3 = pkt_master_to_slave->ref_closure_fingers[2];
                heri_ctrl_msg.percentage_4 = pkt_master_to_slave->ref_closure_fingers[3];
   
                pub.publish(heri_ctrl_msg);

            }
        }
        else {
             //XBot::Logger::error ( "FD_ISSET FALSE\n" );
        }

        ros::spinOnce();
        // TBD check!
        //usleep(1000); // 1 ms
        count++;
    }

    close ( s_recv );
    close ( s_send );
    return 0;
}
