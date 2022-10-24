#ifndef UDP_SERVER_PLUGIN_H
#define UDP_SERVER_PLUGIN_H

#include <xbot2/rt_plugin/control_plugin.h>

class UdpServerPlugin : public XBot::ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void on_start() override;
    void run() override;
    void on_stop() override;
};

#endif // UDP_SERVER_PLUGIN_H
