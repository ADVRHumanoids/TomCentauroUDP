#!/usr/bin/python

from cartesian_interface.pyci_all import *
from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot
import numpy as np
import rospy
import os

scriptdir = os.path.dirname(os.path.abspath(__file__))

def get_xbot_cfg(urdf, srdf):
    cfg = co.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_string_parameter('framework', 'ROS')
    cfg.set_bool_parameter('is_model_floating_base', True)
    return cfg

def update_ik(ci, model, time, dt):
    ci.update(time, dt)
    q = model.getJointPosition()
    qdot = model.getJointVelocity()
    q += qdot * dt
    model.setJointPosition(q)
    model.update()
    return q, qdot

def quintic(alpha):
    if alpha < 0:
        return 0
    elif alpha > 1:
        return 1
    else:
        return ((6*alpha - 15)*alpha + 10)*alpha**3

def compute_h_frame(model, w_T_base, contacts):
    points = np.ones((3, len(contacts)))
    for i, c in enumerate(contacts):
        points[:, i] = model.getPose(c).translation
    points = points - np.mean(points, axis=1, keepdims=True)
    U, S, V = np.linalg.svd(points)
    print(points, U, S, V)
    n = U[:, -1]
    if n[2] < 0:
        n *= -1
    a = w_T_base.linear[:, 0]
    s = np.cross(n, a)  # x ^ z = y
    p = w_T_base.translation


    Th = Affine3()
    print(np.vstack([a, s, n]).T)
    Th.linear = np.vstack([a, s, n]).T
    Th.translation = p
    return Th

rospy.init_node('contact_homing')

cfg = get_xbot_cfg(rospy.get_param('~robot_description'),
                   rospy.get_param('~robot_description_semantic'))

try:
    robot = xbot.RobotInterface(cfg)
    robot.sense()
    robot.setControlMode(xbot.ControlMode.Position())
    ctrlmode = {'j_wheel_{}'.format(i+1) : xbot.ControlMode.Velocity() for i in range(4)}
    ctrlmode['neck_velodyne'] = xbot.ControlMode.Idle()
    ctrlmode['d435_head_joint'] = xbot.ControlMode.Idle()
    robot.setControlMode(ctrlmode)
except:
    robot = None 

model = xbot.ModelInterface(cfg)
if robot is not None:
    model.syncFrom(robot)
    model.update()
rspub = pyci.RobotStatePublisher(model)
rspub.publishTransforms('ci')

contacts = ['wheel_' + str(i+1) for i in range(4)]
w_T_base = model.getPose('base_link')
w_T_h = compute_h_frame(model, w_T_base, contacts)

model.setFloatingBasePose(w_T_h.inverse() * w_T_base)
model.update()
rspub.publishTransforms('ci')
print(model.getPose('base_link'))

dt = 0.01
ikpb = open(scriptdir + '/contact_homing_stack.yaml', 'r').read()
ci = pyci.CartesianInterface.MakeInstance('OpenSot', ikpb, model, dt)

postural = ci.getTask('Postural')
qinit = model.mapToEigen(postural.getReferencePostureMap())
qgoal = model.getRobotState('home')
tf = 5.0


rate = rospy.Rate(1./dt)
time = 0
for i in range(int(tf/dt)):
    tau = time / tf 
    alpha = quintic(tau)
    qref = qinit*(1 - alpha) + qgoal*alpha
    qref = model.eigenToMap(qref)
    postural.setReferencePosture(qref)
    
    q, qdot = update_ik(ci, model, time, dt)
    rspub.publishTransforms('ci')

    if robot is not None:
        robot.setPositionReference(qref)
        robot.setVelocityReference(model.eigenToMap(qdot))
        robot.move()

    time += dt
    rate.sleep()