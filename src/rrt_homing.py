import os
os.environ['XBOT_SEVERITY_LEVEL'] = '2'

from cartesio_planning import planning
from cartesio_planning import validity_check
from cartesio_planning import visual_tools
import numpy as np
import scipy.linalg as la
import yaml
import moveit_msgs.msg 
import rospy
from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
from cartesian_interface.pyci_all import *
import cartesian_interface.roscpp_utils as roscpp

# we need to initialize roscpp (underlying code is in C++!)
cpp_argv = []
if not roscpp.init('centauro_rrt_homing', cpp_argv):
    print('Unable to initialize roscpp node!')

# build model
# get robot description from param server
urdf = rospy.get_param('xbotcore/robot_description')
srdf = rospy.get_param('xbotcore/robot_description_semantic')

# make xbot model
opt = co.ConfigOptions()
opt.set_urdf(urdf)
opt.set_srdf(srdf)
opt.generate_jidmap()
opt.set_bool_parameter('is_model_floating_base', True)
opt.set_string_parameter('model_type', 'RBDL')
opt.set_string_parameter('framework', 'ROS')

model = xbot.ModelInterface(opt)

# goal state is homing
qgoal = model.getRobotState('home')

# get initial state from robot via ros
robot = xbot.RobotInterface(opt)
qref = robot.eigenToMap(robot.getPositionReference())

# define joint range
qmin, qmax = model.getJointLimits()

# we only move the upper body, for now
for i, jname in enumerate(model.getEnabledJointNames()):
    if 'arm' not in jname and jname != 'torso_yaw':
        qmin[i] = qmax[i] = qref.get(jname, 0)
        qgoal[i] = qref.get(jname, 0)
    print(f'{jname} {qmin[i]} {qmax[i]}')


# create planner
yaml_options = '' # leave default options
planner = planning.OmplPlanner(qmin, qmax, yaml_options)

# make planning scene
ps = validity_check.PlanningSceneWrapper(model)

# a validity checker that checks for collisions
def validity_checker(q):
    model.setJointPosition(q)
    model.update()
    ps.update()
    return not ps.checkCollisions()

# start config
qstart = model.mapToEigen(qref)

# check start state is valid
print(f'Start state valid: {validity_checker(qstart)}')
print(ps.getCollidingLinks())

# publish it to a marker array
start_viz = visual_tools.RobotViz(model, 'start', color=[0, 0, 1, 0.5], tf_prefix='ci/')
start_viz.publishMarkers(ps.getCollidingLinks())

# same for goal state...
print(f'Goal state valid: {validity_checker(qgoal)}')
print(ps.getCollidingLinks())
goal_viz = visual_tools.RobotViz(model, 'goal', color=[0, 1, 0, 0.5], tf_prefix='ci/')
goal_viz.publishMarkers(ps.getCollidingLinks())

# execute planner
planner_type = 'LBKPIECE1'
timeout = 5.0
threshold = 0.0

planner.setStartAndGoalStates(qstart, qgoal, threshold)
planner.setStateValidityPredicate(validity_checker)
success = planner.solve(timeout, planner_type)

print(f'planner output : {success}')

solution = np.array(planner.getSolutionPath()).transpose()

# interpolate


# play solution a number of times..
viz = visual_tools.RobotViz(model, 
                            '/solution',
                            color=[0.5, 0, 0.5, 0.5],
                            tf_prefix='ci/')

ntimes = 10
duration = 3.0
dt = duration/solution.shape[1]

for _ in range(ntimes):
    
    for i in range(solution.shape[1]):
        
        q = solution[:, i]
        model.setJointPosition(q)
        model.update()
        viz.publishMarkers()
        rospy.sleep(dt)