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
    }
}

#endif // PACKETS_H

