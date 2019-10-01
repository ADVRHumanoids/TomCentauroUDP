#ifndef PACKETS_H
#define PACKETS_H


namespace TomCentauroUDP {
    namespace packet {
        struct master2slave {
            float timer_master;
            float timer_slave;
            float run;
            
            // operation_mode: 0 cartesian, 1 joint space
            int operation_mode;
            
            // cartesian space
            double r_ref_position_x;
            double r_ref_position_y;
            double r_ref_position_z;
            double r_ref_rotation[9]; // column major ordered
            
            // joint space
            double r_ref_position_1;
            double r_ref_position_2;
            double r_ref_position_3;
            double r_ref_position_4;
            double r_ref_position_5;
            double r_ref_position_6;

            // right hand
            float r_f0_ref_position;
            float r_f1_ref_position;
            float r_f2_ref_position;
            float r_f3_ref_position;

            
        };

        struct slave2master {
            float timer_master;
            float timer_slave;
            float run;
            
            // operation_mode: 0 cartesian, 1 joint space
            int current_operation_mode;
            
            // joint position and torque
            double r_position_1;
            double r_position_2;
            double r_position_3;
            double r_position_4;
            double r_position_5;
            double r_position_6;
            double r_torque_1;
            double r_torque_2;
            double r_torque_3;
            double r_torque_4;
            double r_torque_5;
            double r_torque_6;

            // r is right
            double r_position_x;
            double r_position_y;
            double r_position_z;
            double r_rotation[9]; // column major ordered

            // right hand: position, current, analaogs
            float r_f0_position;
            float r_f1_position;
            float r_f2_position;
            float r_f3_position;
            
            float r_f0_current;
            float r_f1_current;
            float r_f2_current;
            float r_f3_current;

            float r_f0_analogs_1;
            float r_f0_analogs_2;
            float r_f0_analogs_3;
            
            float r_f1_analogs_1;
            float r_f1_analogs_2;
            float r_f1_analogs_3;
            
            float r_f2_analogs_1;
            float r_f2_analogs_2;
            float r_f2_analogs_3;
            
            float r_f3_analogs_1;
            float r_f3_analogs_2;
            float r_f3_analogs_3;
            
            // right F/T
            double force_x;
            double force_y;
            double force_z;
            double torque_x;
            double torque_y;
            double torque_z;
            
        };


        ////////////////////////////////////////////////////////////////////////////////////////////
        /// Teleopman2ToM
        ////////////////////////////////////////////////////////////////////////////////////////////

        struct Teleopman2ToM
        {
        /// packet information
            float timestamp_slave;   // time stamp of this packet
            u_int packet_id;         // packet id

        /// control mode
            int teleopman_mode; // the control mode of the teleop-man: 1 = cartesian impedance control, 2 = joint impedance control, 3. cartesian position/velocity control ....
            int hand_mode;

        /// Cartesian Space EE Feedback
            double position_ee[3];   // current end-effector position of the Teleop-Man: frame{teleop_link5}
            double quaternion_ee[4]; // current end-effector orientation (expressed by quaternion) of the Teleop-Man: frame{teleop_link5}
            double cmd_position_ee[3]; // last commanded position for the IK controller (input to IK solver)
            double cmd_quaternion_ee[4]; // last commanded orientation for the IK controller (input to IK solver)

        /// Joint Space Feedback
            double q[5];             // joint position of the Teleop-Man
            double torque[5];        // joint torque of the Teleop-Man

        /// F/T wrist sensor information
            double force[3];         // the force (Cartesian) sensed by the F/T sensor installed on the wrist
            double torque[3];        // the torque (Cartesian) sensed by the F/T sensor installed on the wrist

        /// Hand information
            double torque_fingers[4]; // the actuator torque for each finger. (Do we have 4 fingers?)
            double q_fingers[4];      // the actuator position for each finger.
            double position_tip_fingers1[3]; // the cartesian tip position of the first finger (Thumb)
            double position_tip_fingers2[3]; // the cartesian tip position of the second finger (index)
            double position_tip_fingers3[3]; // the cartesian tip position of the third finger (middle)
            double position_tip_fingers4[3]; // the cartesian tip position of the fourth finger (ring?)

        /// Pressure hand sensor information
            double pressure_fingers[11];    // the pressure sensor data installed on each phalanx
        };


        ////////////////////////////////////////////////////////////////////////////////////////////
        /// ToM2Teleopman
        ////////////////////////////////////////////////////////////////////////////////////////////

        struct ToM2Teleopman
        {
        /// packet information
            float timestamp_slave;   // time stamp of this packet
            u_int packet_id;         // packet id

        /// Cartesian Space EE Feedback
            double position_ee[3];   // end-effector position of the Teleop-Man: frame{teleop_link5}
            double quaternion_ee[4]; // end-effector orientation of the Teleop-Man. expressed in quaternion

        /// Joint Space Feedback
            double q[5];             // joint position of the Teleop-Man
            double torque[5];        // joint torque of the Teleop-Man

        /// Hand information
            double closure_cmd_fingers[4];  // how do we control the fingers?

        };
    }
}
#endif // PACKETS_H

