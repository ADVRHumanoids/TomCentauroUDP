#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>

namespace tom_centauro_udp { namespace packet {


/**
 * @brief Command from master (teleop device) to slave (robot)
 */
struct __attribute__((packed)) master2slave
{
    float timer_master;
    float timer_slave;
    bool run;  // it true, cmd is actually sent to ik

    // ee URDF name
    char ee_id[16];  // up to 15 chars + null-termination

    // ee pose command
    float position_x;
    float position_y;
    float position_z;
    float rotation[9];  // column major ordering

    // hand command (desired pos, desired force?)
    float gripper_pos;
    float gripper_force;

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
    bool run;  // what is this for ?

    // ee URDF name
    char ee_id[16];  // up to 15 chars + null-termination

    // ee pose feedback
    float position_x;
    float position_y;
    float position_z;
    float rotation[9];  // column major ordering

    // hand feedback
    float gripper_pos;
    float gripper_force;

    // simple check code
    uint32_t magic_code = expected_magic_code;

    static constexpr uint32_t expected_magic_code = 0xbadf00d;
};

} }

#endif // PACKETS_H

