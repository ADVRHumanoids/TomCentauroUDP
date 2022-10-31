#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>

namespace tom_centauro_udp { namespace packet {


/**
 * @brief Command from master (teleop device) to slave (robot)
 */
struct __attribute__((packed)) master2slave
{
    float timer_master = 0;
    float timer_slave = 0;
    bool run = false;  // should slave execute this command?

    // ee URDF name
    char ee_id[16];  // up to 15 chars + null-termination

    // if true, velocity commands are sent, and position
    // is disregarded (useful for moving the base)
    bool velocity_ctrl = false;

    // ee pose command
    float position_x = 0;
    float position_y = 0;
    float position_z = 0;
    float rotation[4];  // quaternion (w, xi, yj, zk)

    // hand command (desired pos, desired force?)
    float gripper_pos = 0;
    float gripper_force = 0;
    
    // velocity commands
    bool jst_button[3];  // used for what ?
    float vel_xy[2];  // raw joystick x,y data [-1 to 1]
    float vel_yaw;  // possible centauro heading command (rotatate about z axis)
    float aux_pos[2];  // possible centauro velocity/direction command (x,y)

    // simple check code
    uint32_t magic_code = expected_magic_code;

    static constexpr uint32_t expected_magic_code = 0xdeadbeef;
};



/**
 * @brief Feedback from robot
 */
struct __attribute__((packed)) slave2master
{
    float timer_master;
    float timer_slave;
    bool run;  // master is executing commands

    // ee URDF name
    char ee_id[16];  // up to 15 chars + null-termination

    // ee pose feedback
    float position_x;
    float position_y;
    float position_z;
    float rotation[4];      // quaternion (w, xi, yj, zk)

    // hand feedback
    float gripper_pos;
    float gripper_force;


    // jostick commands
    float aux_heading;      // posiible centauro heading state (rotatate about z axis)
    float aux_pos[2];       // posible centtaoro velocity/direction state (x,y)

    // simple check code
    uint32_t magic_code = expected_magic_code;

    static constexpr uint32_t expected_magic_code = 0xbadf00d;
};

} }

#endif // PACKETS_H

