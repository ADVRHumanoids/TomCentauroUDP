#include "udp_server_plugin.h"

bool UdpServerPlugin::on_initialize()
{
    // bind on all interfaces, on given port
    auto addr = getParamOr<std::string>("~addr", "0.0.0.0");

    int port = getParamOr("port", 8081);

    if(!_srv.bind(addr, port))
    {
        throw std::runtime_error("udp server bind failed");
    }

    _srv.set_timeout_sec(0.1);

    // subscribe to ik heartbeat
    _heartbeat_sub = subscribe<bool>("/ik_plugin/heartbeat",

        [this](const bool& msg)
        {
            _last_heartbeat_recv_time = chrono::steady_clock::now();
        }, 1, &_queue);

    return true;
}

void UdpServerPlugin::on_start()
{
    // consume stale messages
    tom_centauro_udp::packet::master2slave packet_to_robot;
    while(_srv.try_receive(packet_to_robot));

    _nmsgs = 0;
    _nrepl = 0;
    _start_time = chrono::steady_clock::now();
}

void UdpServerPlugin::run()
{
    // spin ros callbacks to get current reference
    _queue.run();

    // see if we got ik heartbeat
    if(chrono::steady_clock::now() - _last_heartbeat_recv_time > 500ms)
    {
        jerror("ik not running, aborting");
        abort();
        return;
    }

    tom_centauro_udp::packet::master2slave packet_to_robot;

    // blocking receive
    bool timeout_expired = false;
    if(!_srv.receive(packet_to_robot, &timeout_expired))
    {
        if(timeout_expired)
        {
            return;
        }
        else
        {
            jerror("failed to receive packet to robot");
            return;
        }
    }

    _nmsgs++;

    // consume queue
    while(_srv.try_receive(packet_to_robot))
    {
        _nmsgs++;
    }

    // periodically print statistics
    XBOT2_INFO_EVERY(2.0s, "recv {}  repl {} (client at {})",
                     _nmsgs, _nrepl, _srv.get_last_client_address());

    // if this plugin has just started, we are likely receiving messages
    // from a stale client -> abort
    if(chrono::steady_clock::now() - _start_time < 500ms)
    {
        jerror("message received within 500ms from start time, aborting");
        abort();
        return;
    }

    // safety checks
    if(!tom_centauro_udp::check_pkt_valid(packet_to_robot))
    {
        jerror("invalid packet received");
        return;
    }



    // get end effector name
    std::string ee_id = packet_to_robot.ee_id;

    // retrieve or create publisher and subscriber
    CommmandPublisherPtr pub;

    {
        auto pub_it = _ee_pubs.find(ee_id);

        if(pub_it == _ee_pubs.end())
        {
            jinfo("creating publisher for ee '{}'", ee_id);

            pub = advertise<Eigen::Affine3d>("/ik_plugin/" + ee_id + "/command");

            _ee_pubs[ee_id] = pub;

        }
        else
        {
            pub = pub_it->second;
        }
    }

    if(_ee_subs.count(ee_id) == 0)
    {
        // state  subscriber
        auto sub = subscribe<Eigen::Affine3d>(
            "/ik_plugin/" + ee_id + "/state",
            [ee_id, this](const Eigen::Affine3d& msg)
            {
                _ee_state[ee_id] = msg;
            },
            1,
            &_queue);

        _ee_subs[ee_id] = sub;
    }



    // reply to client with current reference if we got one
    tom_centauro_udp::packet::slave2master packet_to_teleop;

    auto ref_it = _ee_state.find(ee_id);

    if(ref_it != _ee_state.end())
    {
        tom_centauro_udp::fill_pkt_ee_id(packet_to_teleop, ee_id);

        tom_centauro_udp::fill_pkt_with_pose(packet_to_teleop,
                                             ref_it->second);

        _srv.reply(packet_to_teleop);

        _nrepl++;
    }
    else
    {
        jerror("state for task '{}' unavailable", ee_id);
    }

    // should we pub this reference?
    if(packet_to_robot.run)
    {
        Eigen::Affine3d pose;

        tom_centauro_udp::get_pose_from_pkt(packet_to_robot,
                                            pose);

        pub->publish(pose);

        // gripper (TODO)
        //        sensor_msgs::JointState msg;
        //        msg.name = {"gripper_joint"};
        //        msg.position = {clamp(packet_to_robot.gripper_pos)};
        //        gripper_pub.publish(msg);
    }
}

void UdpServerPlugin::on_stop()
{
}

XBOT2_REGISTER_PLUGIN(UdpServerPlugin, tom_centauro_udp_server);
