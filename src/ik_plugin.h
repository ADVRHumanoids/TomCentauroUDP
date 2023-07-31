#ifndef IKPLUGIN_H
#define IKPLUGIN_H

#include <xbot2/rt_plugin/control_plugin.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Cartesian.h>

using namespace XBot;
using namespace XBot::Cartesian;

class IkPlugin : public XBot::ControlPlugin
{

public:

    using ControlPlugin::ControlPlugin;

    bool on_initialize() override;
    void on_start() override;
    void run() override;
    void on_stop() override;

private:

    ModelInterface::Ptr _model;
    CartesianInterfaceImpl::Ptr _ci;

    CallbackQueue _queue;

    std::vector<std::function<void(void)>> _fn;

    double _fake_time;

    JointIdMap _qmap;
    Eigen::VectorXd _q, _qdot;


};

#endif // IKPLUGIN_H
