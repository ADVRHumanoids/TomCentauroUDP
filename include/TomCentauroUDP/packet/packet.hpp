#ifndef PACKETS_H
#define PACKETS_H

namespace TomCentauroUDP {
    namespace packet {

        ////////////////////////////////////////////////////////////////////////////////////////////
        /// Teleopman2ToM
        ////////////////////////////////////////////////////////////////////////////////////////////

        //struct __attribute__((__packed__))Teleopman2ToM
        struct Teleopman2ToM
        {
        /// packet information
            float timestamp_master;   // time stamp of this packet
            float timestamp_slave;
            int packet_id;           // packet id

        /// actual control mode
            int teleopman_mode; // operation_mode: 0 cartesian, 1 joint space
            int hand_mode;

        /// Cartesian Space EE Feedback
            double position_ee[3];   // current end-effector position of the Teleop-Man: frame{teleop_link5}
            double quaternion_ee[4]; // current end-effector orientation (expressed by quaternion) of the Teleop-Man: frame{teleop_link5}
            double ref_position_ee[3]; // last commanded position for the IK controller (input to IK solver)
            double ref_quaternion_ee[4]; // last commanded orientation for the IK controller (input to IK solver)

        /// Joint Space Feedback
            double q[5];             // joint position of the Teleop-Man
            double torque[5];        // joint torque of the Teleop-Man

        /// F/T wrist sensor information
            double ft_force[3];         // the force (Cartesian) sensed by the F/T sensor installed on the wrist
            double ft_torque[3];        // the torque (Cartesian) sensed by the F/T sensor installed on the wrist

        /// Hand information
            double i_fingers[4]; // the actuator current (in Amper) for each finger.
            double q_fingers[4];      // the actuator position for each finger.

        /// Pressure hand sensor information
            double pressure_fingers[12];    // the pressure sensor data installed on each phalanx
        }; 


        ////////////////////////////////////////////////////////////////////////////////////////////
        /// ToM2Teleopman
        ////////////////////////////////////////////////////////////////////////////////////////////

        //struct __attribute__((__packed__))ToM2Teleopman
        struct ToM2Teleopman
        {
        /// packet information
            float timestamp_master;   // time stamp of this packet
            float timestamp_slave; 
            int packet_id;            // packet id

        /// Cartesian Space EE Feedback
            double ref_position_ee[3];   // end-effector position of the Teleop-Man: frame{teleop_link5}
            double ref_quaternion_ee[4]; // end-effector orientation of the Teleop-Man. expressed in quaternion

        /// Joint Space Feedback
            double ref_q[5];             // joint position of the Teleop-Man
            double ref_torque[5];        // joint torque of the Teleop-Man

        /// Hand information
            double ref_closure_fingers[4];  // how do we control the fingers? NOTE Percentage

        };
    }
}
#endif // PACKETS_H

