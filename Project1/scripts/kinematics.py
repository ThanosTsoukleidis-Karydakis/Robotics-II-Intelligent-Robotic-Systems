#!/usr/bin/env python3

"""
Compute state space kinematic matrices for xArm7 robot arm (5 links, 7 joints)
"""

import numpy as np
import rospy

class xArm7_kinematics():
    def __init__(self):

        self.l1 = 0.267
        self.l2 = 0.293
        self.l3 = 0.0525
        self.l4 = 0.3512
        self.l5 = 0.1232

        self.theta1 = 0.2225 #(rad) (=12.75deg)
        self.theta2 = 0.6646 #(rad) (=38.08deg)

        pass

    def compute_jacobian(self, r_joints_array):
        # Computing the Jacobian matrix (with respect to the alternative DH convention)
        b = np.matrix([0,0,1])

        J_L1 =  np.cross(b,np.transpose(self.tf_A07(r_joints_array)[0:3,3]))
        J_L2 =  np.cross(np.transpose(self.tf_A02(r_joints_array)[0:3,2]),np.transpose(np.subtract(self.tf_A07(r_joints_array)[0:3,3],self.tf_A01(r_joints_array)[0:3,3])))
        J_L3 =  np.cross(np.transpose(self.tf_A03(r_joints_array)[0:3,2]),np.transpose(np.subtract(self.tf_A07(r_joints_array)[0:3,3],self.tf_A02(r_joints_array)[0:3,3])))
        J_L4 =  np.cross(np.transpose(self.tf_A04(r_joints_array)[0:3,2]),np.transpose(np.subtract(self.tf_A07( r_joints_array)[0:3,3],self.tf_A03(r_joints_array)[0:3,3])))
        J_L5 =  np.cross(np.transpose(self.tf_A05(r_joints_array)[0:3,2]),np.transpose(np.subtract(self.tf_A07(r_joints_array)[0:3,3],self.tf_A04(r_joints_array)[0:3,3])))
        J_L6 =  np.cross(np.transpose(self.tf_A06(r_joints_array)[0:3,2]),np.transpose(np.subtract(self.tf_A07(r_joints_array)[0:3,3],self.tf_A05(r_joints_array)[0:3,3])))
        J_L7 =  np.cross(np.transpose(self.tf_A07(r_joints_array)[0:3,2]),np.transpose(np.subtract(self.tf_A07(r_joints_array)[0:3,3],self.tf_A06(r_joints_array)[0:3,3])))


        J_11 = J_L1[0,0]
        J_12 = J_L2[0,0]
        J_13 = J_L3[0,0]
        J_14 = J_L4[0,0]
        J_15 = J_L5[0,0]
        J_16 = J_L6[0,0]
        J_17 = J_L7[0,0]

        J_21 = J_L1[0,1]
        J_22 = J_L2[0,1]
        J_23 = J_L3[0,1]
        J_24 = J_L4[0,1]
        J_25 = J_L5[0,1]
        J_26 = J_L6[0,1]
        J_27 = J_L7[0,1]

        J_31 = J_L1[0,2]
        J_32 = J_L2[0,2]
        J_33 = J_L3[0,2]
        J_34 = J_L4[0,2]
        J_35 = J_L5[0,2]
        J_36 = J_L6[0,2]
        J_37 = J_L7[0,2]



        J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],
                        [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],
                        [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ]])
        return J

    # computing each A matrix using the forward kinematics analysis
    def tf_A01(self, r_joints_array):
        tf = np.matrix([[np.cos(r_joints_array[0]) , -np.sin(r_joints_array[0]) , 0 , 0],\
                        [np.sin(r_joints_array[0]) , np.cos(r_joints_array[0]) , 0 , 0],\
                        [0 , 0 , 1 , self.l1],\
                        [0 , 0 , 0 , 1]])
        return tf

    def tf_A02(self, r_joints_array):
        tf_A12 = np.matrix([[np.cos(r_joints_array[1]) , -np.sin(r_joints_array[1]) , 0 , 0],\
                            [0 , 0 , 1 , 0],\
                            [-np.sin(r_joints_array[1]) , -np.cos(r_joints_array[1]) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot(self.tf_A01(r_joints_array), tf_A12 )
        return tf

    def tf_A03(self, r_joints_array):
        tf_A23 = np.matrix([[np.cos(r_joints_array[2]) , -np.sin(r_joints_array[2]) , 0 , 0],\
                            [0 , 0 , -1 , -self.l2],\
                            [np.sin(r_joints_array[2]) , np.cos(r_joints_array[2]) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
        return tf

    def tf_A04(self, r_joints_array):
        tf_A34 = np.matrix([[np.cos(r_joints_array[3]) , -np.sin(r_joints_array[3]) , 0 , self.l3],\
                            [0 , 0 , -1 , 0],\
                            [np.sin(r_joints_array[3]) , np.cos(r_joints_array[3]) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
        return tf

    def tf_A05(self, r_joints_array):
        tf_A45 = np.matrix([[np.cos(r_joints_array[4]) , -np.sin(r_joints_array[4]) , 0 , (self.l4)*np.sin(self.theta1)],\
                            [0 , 0 , -1 , -(self.l4)*np.cos(self.theta1)],\
                            [np.sin(r_joints_array[4]) , np.cos(r_joints_array[4]) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
        return tf

    def tf_A06(self, r_joints_array):
        tf_A56 = np.matrix([[np.cos(r_joints_array[5]) , -np.sin(r_joints_array[5]) , 0 , 0],\
                            [0 , 0 , -1 , 0],\
                            [np.sin(r_joints_array[5]) , np.cos(r_joints_array[5]) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
        return tf

    def tf_A07(self, r_joints_array):
        tf_A67 = np.matrix([[np.cos(r_joints_array[6]) , -np.sin(r_joints_array[6]) , 0 , (self.l5)*np.sin(self.theta2)],\
                            [0 , 0 , 1 , (self.l5)*np.cos(self.theta2)],\
                            [-np.sin(r_joints_array[6]) , -np.cos(r_joints_array[6]) , 0 , 0],\
                            [0 , 0 , 0 , 1]])
        tf = np.dot( self.tf_A06(r_joints_array), tf_A67)
        return tf
