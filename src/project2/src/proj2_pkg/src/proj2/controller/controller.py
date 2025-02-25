#!/usr/bin/env python

"""
Starter code for EECS C106B Spring 2020 Project 2.
Author: Amay Saxena
"""
import numpy as np
import sys

#ADDED 
import cvxpy as cp
import sympy as sp

import tf2_ros
import tf
from std_srvs.srv import Empty as EmptySrv
import rospy
from proj2_pkg.msg import BicycleCommandMsg, BicycleStateMsg
from proj2.planners import SinusoidPlanner, RRTPlanner, BicycleConfigurationSpace

class BicycleModelController(object):
    def __init__(self):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe)
        self.state = BicycleStateMsg()
        rospy.on_shutdown(self.shutdown)

    def execute_plan(self, plan):
        """
        Executes a plan made by the planner

        Parameters
        ----------
        plan : :obj: Plan. See configuration_space.Plan
        """
        if len(plan) == 0:
            return
        rate = rospy.Rate(int(1 / plan.dt))
        start_t = rospy.Time.now()
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_t).to_sec()
            if t > plan.times[-1] : #changed
                break
            state, cmd = plan.get(t)
            self.step_control(state, cmd)
            rate.sleep()

        #ADDED
        #if np.linalg.norm(self.state[:2] - target_position)
        self.cmd([0, 0])

    def step_control(self, target_position, open_loop_input):
        """Specify a control law. For the grad/EC portion, you may want
        to edit this part to write your own closed loop controller.
        Note that this class constantly subscribes to the state of the robot,
        so the current configuratin of the robot is always stored in the 
        variable self.state. You can use this as your state measurement
        when writing your closed loop controller.

        Parameters
        ----------
            target_position : target position at the current step in
                              [x, y, theta, phi] configuration space.
            open_loop_input : the prescribed open loop input at the current
                              step, as a [u1, u2] pair.
        Returns:
            None. It simply sends the computed command to the robot.
        """
        #implementing controller from paper "realizing simultaneous lane keepign and adaptive speed regulation on 
        # accesible mobile robot testbeds", while changing the model from unicycle to bicycle
        x, y, theta, phi = self.state
        x_d, y_d, theta_d, phi_d = target_position
        #v = open_loop_input[0]

        #v_output = v_d
        v_output = open_loop_input[0]
        omega_delta_output = open_loop_input[1]

        #constants
        d_max = 0.1 #max distance lane
        tau = 0.002 #time headway in seconds
        D = 0.1 #distance between cars
        gamma = 1 #cbf scaling
        a_max = 2.0 
        L = 0.1

        v_d = 0.43 #desired velocity

        v = cp.Variable()
        omega_delta = cp.Variable()

        #tuning vars
        delta_1 = cp.Variable(nonneg=True) #relaxation vars
        delta_2 = cp.Variable(nonneg=True)
        delta_3 = cp.Variable(nonneg=True)

        p1 = 0.01
        p2 = 0.05
        p3 = 0.01 
        p4 = 0.01 
        p5 = 0.01

        c1 = 0.05 #what should these be
        c2 = 0.2
        c3 = 0.5

        #CBF Constraints
        h_asr = D - tau * v
        #use the linear version just because we dont have the best hardware
        h_lk = d_max - cp.abs(y) - (1/2) * (v*np.tan(phi))**2/a_max 

        print("h_asr: ", h_asr)
        print("h_lk: ", h_lk)
        #CLF Constraints
        V_1 = (v - v_d)**2
        V_2 = (theta - theta_d)**2 #angular velocity
        V_3 = (x - x_d)**2 + (y - y_d)**2 #changes from the paper implementation b/c using the bicycle model and only need to track position

        V_1_dot = 2 * (v - v_d) #* a #times vdot
        V_2_dot = 2 * (theta - theta_d) * (v/L) * np.tan(phi)
        V_3_dot = 2 * (x - x_d) * v * np.cos(theta) + 2 * (y - y_d) * v * np.sin(theta)
        
        u = cp.vstack([v, omega_delta, delta_1, delta_2, delta_3])
        H = cp.diag([p1, p2, p3, p4, p5])
        
        cost = cp.quad_form(u, H)

        objective = cp.Minimize(cost)
        constraints = [
            #v + (gamma/tau) * h_asr >= 0
            v  >= 0, #do we need this? theres no car in front
            #omega_delta + h_lk >= 0, 
            V_1_dot + c1*V_1 <= delta_1,
            V_2_dot + c2*V_2 <= delta_2,
            V_3_dot + c3*V_3 <= delta_3 
        ]

        prob = cp.Problem(objective, constraints)
        result = prob.solve()

        if v.value is not None:
            v_output = v.value
        if omega_delta.value is not None:
            omega_delta_output = omega_delta.value
        
        print("vel:", v_output)
        print("w_d:", omega_delta_output)
        self.cmd([ v_output, omega_delta_output])
        #self.cmd(open_loop_input)

    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim

        Parameters
        ----------
        msg : numpy.ndarray
        """
        self.pub.publish(BicycleCommandMsg(*msg))

    def subscribe(self, msg):
        """
        callback fn for state listener.  Don't call me...
        
        Parameters
        ----------
        msg : :obj:`BicycleStateMsg`
        """
        self.state = np.array([msg.x, msg.y, msg.theta, msg.phi])

    def shutdown(self):
        rospy.loginfo("Shutting Down")
        self.cmd((0, 0))
