#include "ik_plugin.h"


bool IkPlugin::on_initialize()
{
    // get ik problem from ros param
    std::string ik_str = getParamOrThrow<std::string>("~problem_description/content");

    auto ik_yaml = YAML::Load(ik_str);

    // create model and ci for rt loop
    _model = ModelInterface::getModel(_robot->getConfigOptions());

    auto rt_ctx = std::make_shared<Cartesian::Context>(
        std::make_shared<Parameters>(getPeriodSec()),
        _model);

    ProblemDescription ik_problem(ik_yaml, rt_ctx);

    auto impl_name = getParamOr<std::string>("~solver", "OpenSot");

    _ci = CartesianInterfaceImpl::MakeInstance(impl_name, ik_problem, rt_ctx);

    _ci->update(0, 0);

    // detect cartesian tasks
    auto to_cartesian = [](auto t)
    {
        return std::dynamic_pointer_cast<CartesianTask>(t);
    };

    for(auto tname : _ci->getTaskList())
    {
        auto t = _ci->getTask(tname);

        auto cart = to_cartesian(t);

        if(!cart)
        {
            continue;
        }

        jinfo("detected cartesian task with name '{}'",
              tname);

        // define state publisher function
        auto state_pub = advertise<Eigen::Affine3d>(fmt::format("~/{}/state", tname));

        auto pub_state_fn = [this, cart, state_pub]()
        {
            Eigen::Affine3d T;
            cart->getPoseReference(T);
            state_pub->publish(T);
        };

        _fn.push_back(pub_state_fn);

        // define command subscriber callback
        auto sub_ref_fn = [this, cart](const Eigen::Affine3d& msg)
        {
//            jinfo("{} received pose: {} {}",
//                  cart->getName(),
//                  msg.translation().transpose(),
//                  Eigen::Quaterniond(msg.linear()).coeffs().transpose());

            cart->setPoseReference(msg);
        };

        subscribe<Eigen::Affine3d>(fmt::format("~/{}/command", tname),
                                   sub_ref_fn,
                                   1,
                                   &_queue);

    }

    // heartbeat publisher
    auto pub = advertise<bool>("~/heartbeat");

    auto pub_hb_fn = [this, pub]()
    {
        pub->publish(true);
    };

    _fn.push_back(pub_hb_fn);


    // we may want to enable online trj generation
    // (i.e. enforcement of vel/acc limits at the ee reference generation layer)
    if(getParamOr("~enable_otg", false))
    {
        _ci->enableOtg(rt_ctx->params()->getControlPeriod());
        jinfo("otg enabled");

        // TBD: set otg limits from param

    }

    // declare position ctrl mode
    setDefaultControlMode(ControlMode::Position());

    return true;

}

void IkPlugin::on_start()
{
    // we use a fake time, and integrate it by the expected dt
    _fake_time = 0;

    // update the model
    _robot->sense(false);
    _robot->getPositionReference(_qmap);
    _model->setJointPosition(_qmap);
    _model->update();

    // reset ci
    _ci->reset(_fake_time);
}

void IkPlugin::run()
{
    // spin subscribers
    _queue.run();

    // solve ik
    if(!_ci->update(_fake_time, getPeriodSec()))
    {
        jerror("unable to solve \n");
        return;
    }

    // integrate model with obtained solution
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _q += getPeriodSec() * _qdot;
    _model->setJointPosition(_q);
    _model->update();

    // update time
    _fake_time += getPeriodSec();

    // set reference to robot and move
    _robot->setReferenceFrom(*_model);
    _robot->move();

    // run publishers
    for(auto& fn : _fn)
    {
        fn();
    }
}

void IkPlugin::on_stop()
{

}

XBOT2_REGISTER_PLUGIN(IkPlugin, tom_centauro_ik)
