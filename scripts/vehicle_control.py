#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Prateek Parmeshwar
Email   : pparmesh@andrew.cmu.edu
Date    : Oct 23, 2019
'''

import os
import math
import numpy as np
import matplotlib.pyplot as plt

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from delta_prediction.msg import EgoStateEstimate, EgoStateEstimateArray


class VehicleControl:
    '''Planning and Controls Subsystem'''
    def __init__(self, ctrl_freq, ttc):
        self.ctrl_freq = ctrl_freq
        self.ttc = ttc
        self.current_state = EgoStateEstimate()
        
    # Method to generate reference trajectory
    def generate_evasive_traj(self,pub_vis):
        ''' Generates a minimum jerk trajectory in x and y'''
        # boundary_vals = ego_state_msg.pose.position.x
        # boundary_vals = ego_state_msg.pose.position.x
        # xi,yi,xf,yf,vxi,vyi,axi,ayi = boundary_vals
        xi = self.current_state.pose.position.x
        yi = self.current_state.pose.position.y
        xf = self.current_state.pose.position.x + 4
        yf = self.current_state.pose.position.y + 1

        vxi = self.current_state.twist.linear.x
        vyi = self.current_state.twist.linear.y

        axi = 0
        ayi = 0



        x_traj = [] # List of list of position and velocity at ctrl_freq in x
        y_traj = [] # List of list of position and velocity at ctrl_freq in y

        # minimum jerk trajectory is a 5th order polynomial
        # y = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        # Given initial and final values in pos, vel and acc (Note final acc is 0 and final vel is 0) coeffs are:
        T = self.ttc
        a0x = xi
        a1x = vxi
        a2x = axi/2
        a3x = -(20*xi - 20*xf + 12*T*vxi + 11*axi*T**2 - 8*axi*T)/(2*T**3)
        a4x = (30*xi - 30*xf + 16*T*vxi + 17*axi*T**2 - 14*axi*T)/(2*T**4)
        a5x = -(12*xi - 12*xf + 6*T*vxi + 7*axi*T**2 - 6*axi*T)/(2*T**5)
        # Similarly for y
        a0y = yi
        a1y = vyi
        a2y = ayi/2
        a3y = -(20*yi - 20*yf + 12*T*vyi + 11*ayi*T**2 - 8*ayi*T)/(2*T**3)
        a4y = (30*yi - 30*yf + 16*T*vyi + 17*ayi*T**2 - 14*ayi*T)/(2*T**4)
        a5y = -(12*yi - 12*yf + 6*T*vyi + 7*ayi*T**2 - 6*ayi*T)/(2*T**5)

        dt = 1/self.ctrl_freq # Define time step
        t = dt # Current time
        # Populate trajectory information
        i=0
        while t <= self.ttc:
            # Trajectory in x
            x_t = a0x + a1x*t + a2x*t**2 + a3x*t**3 + a4x*t**4 + a5x*t**5
            vx_t = a1x + 2*a2x*t + 3*a3x*t**2 + 4*a4x*t**3 + 5*a5x*t**4
            x_traj.append([x_t, vx_t])
            # Trajectory in y
            y_t = a0y + a1y*t + a2y*t**2 + a3y*t**3 + a4y*t**4 + a5y*t**5
            vy_t = a1y + 2*a2y*t + 3*a3y*t**2 + 4*a4y*t**3 + 5*a5y*t**4
            y_traj.append([y_t, vy_t])
            i = i+1
            t = t + dt# Increment time
              
        x_traj = np.asarray(x_traj)
        y_traj = np.asarray(y_traj)

        self.publish_ref_traj(x_traj,y_traj,pub_vis)
    
    # ROS Callback to get current state
    def traj_gen_callback(self, ego_state_msg):
        self.current_state = ego_state_msg


    # Publish reference trajectory
    def publish_ref_traj(self, x_traj, y_traj,pub):
        pos_vals = np.c_[x_traj[:,0],y_traj[:,0]]
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id
        marker.id = marker_id
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        for x, y in trajectory:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.5
            marker.points.append(point)

        marker.scale.x = 0.15
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        pub.publish(marker)

    
    
    
    
 def main():
     rospy.init_node('delta_vehicle_control',anonymous=True)
     ctrl_freq = 10
     ttc = 2
     delta_control = VehicleControl(ctrl_freq,ttc) # Instantiate class object
     rospy.Subscriber('/delta/prediction/ego_vehicle/state',EgoStateEstimate,delta_control.traj_gen_callback) #ROS Callback
     pub_vis = rospy.Publisher('/delta/planning_controls/evasive_traj',Marker,queue_size=10)
     r = rospy.Rate(10)

     while not rospy.is_shutdown():
        delta_control.generate_evasive_traj(pub_vis)
        r.sleep()






if __name__ == "__main__":
    main()