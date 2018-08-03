#ifndef PACKETS_H
#define PACKETS_H


namespace TomCentauroUDP {
    namespace packet {
        struct master2slave {
            float timer_master;
            float timer_slave;
            float run;

            // r is right
            double r_position_x;
            double r_position_y;
            double r_position_z;
            double r_rotation[9]; // column major ordered

            // right hand
            float r_f0_position;
            float r_f1_position;
            float r_f2_position;
            float r_f3_position;

            float r_f0_pressure;
            float r_f1_pressure;
            float r_f2_pressure;
            float r_f3_pressure;
        };

        struct slave2master {
            float timer_master;
            float timer_slave;
            float run;

            // 1 is right
            double r_position_x;
            double r_position_y;
            double r_position_z;
            double r_rotation[9]; // column major ordered

            // right hand
            float r_f0_position;
            float r_f1_position;
            float r_f2_position;
            float r_f3_position;

            float r_f0_pressure;
            float r_f1_pressure;
            float r_f2_pressure;
            float r_f3_pressure;
        };
    }
}

#endif // PACKETS_H

