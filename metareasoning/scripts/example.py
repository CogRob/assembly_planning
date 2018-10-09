#!/usr/bin/env python
import roslib
roslib.load_manifest('dmp')
import rospkg
import rospy
import numpy as np
from dmp.srv import *
from dmp.msg import *
import pickle


# Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, D_gain, num_bases):
    demotraj = DMPTraj()

    for i in range(len(traj)):
        pt = DMPPoint()
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt * i)

    k_gains = [K_gain] * dims
    d_gains = [D_gain] * dims

    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    print "LfD done"

    return resp


# Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt,
                    integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt,
                   integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    print "DMP planning done"

    return resp


if __name__ == '__main__':
    rospy.init_node('dmp_tutorial_node')
    rp = rospkg.RosPack()
    path = rp.get_path('metareasoning')
    trajectory_file = path + '/data/trajectory'

    #Create a DMP from a 2-D trajectory
    dims = 8
    dt = 0.1
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 20

    # parse trajectory file
    i = 0
    traj = []
    with open(trajectory_file, 'rt') as f:
        for line in f:
            if i == 0:
                # skip header
                i += 1
            else:
                traj.append([float(x) for x in line.split(',')[1:]])
    print traj

    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    # Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    # Now, generate a plan
    x_0 = [-0.27, 1.60, 0.34, -1.15, 0.06, 0.95, 0.38]  # TODO replace
    x_dot_0 = [0.0] * dims
    t_0 = 0
    goal = [-0.27, 1.90, 0.33, -1.27, 0.06, 1.00, 0.38]  # TODO replace
    goal_thresh = [0.01] * dims
    seg_length = -1  # Plan until convergence to goal
    tau = resp.tau
    dt = 0.1
    integrate_iter = 1
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length,
                           tau, dt, integrate_iter)

    with open(path + '/data/adapted_plan.pkl', 'wb') as f:
        pickle.dump(plan, f)
    print plan
