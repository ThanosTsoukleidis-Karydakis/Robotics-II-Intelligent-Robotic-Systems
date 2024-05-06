#!/usr/bin/env python3

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

from scipy.spatial import distance


# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):
        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()

        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        # gazebo model's states
        self.model_states = ModelStates()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)
        # Obstacles
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of joint 1 is stored in :: self.joint_states.position[0])

    def model_states_callback(self, msg):
        # ROS callback to get the gazebo's model_states

        self.model_states = msg
        # (e.g. #1 the position in y-axis of GREEN obstacle's center is stored in :: self.model_states.pose[1].position.y)
        # (e.g. #2 the position in y-axis of RED obstacle's center is stored in :: self.model_states.pose[2].position.y)

    def publish(self):

        # set configuration
        # total pitch: j2-j4+j6+pi (upwards: 0rad)
        j2 = 0.7 ; j4 = np.pi/2
        j6 = - (j2-j4)
        self.joint_angpos = [0, j2, 0, j4, 0, j6, 0]
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        # The time (in seconds) when the simulation begins.
        start = rostime_now.secs + rostime_now.nsecs/1000000000

        # Some useful initializations for matrices that we are going to need in order
        # to make plots
        t_now_points = []
        distanceObs1_Joint3 = []
        distanceObs1_Joint4 = []
        distanceObs1_Joint5 = []
        distanceObs2_Joint3 = []
        distanceObs2_Joint4 = []
        distanceObs2_Joint5 = []
        velocity_error_array = []
        desired_velocity_array = []
        velocity_array = []
        joints_positions = [[] for i in range(7)]
        joints_velocities = [[] for i in range(7)]
        x_pos = []
        y_pos = []
        z_pos = []

        # boolean variable that becomes false when the plots have been created and saved
        done = True
        # the time of the previous iteration (needed for some derivatives)
        t_prev = start
        # initialization of previous x, y, z position coordinates of the end effector
        prev_x = 0.617
        prev_y = 0.0
        prev_z = 0.199


        while not rospy.is_shutdown():
            # Reading the position (x, y) of the two obstacles
            obstacle1 = (self.model_states.pose[1].position.x, self.model_states.pose[1].position.y)
            obstacle2 = (self.model_states.pose[2].position.x, self.model_states.pose[2].position.y)
            # the time of the current iteration
            rostime_now = rospy.get_rostime()
            now = rostime_now.secs + rostime_now.nsecs/1000000000

            # the time that has passed from the start of the simulation
            t_now = now-start

            # Compute each transformation matrix wrt the base frame from joints' angular positions
            self.A01 = self.kinematics.tf_A01(self.joint_angpos)
            # Compute the current position of some joints
            p_0E = self.kinematics.tf_A07(self.joint_angpos)[0:3,3]
            P0_4 = self.kinematics.tf_A04(self.joint_angpos)[0:3,3]
            P0_5 = self.kinematics.tf_A05(self.joint_angpos)[0:3,3]
            P0_3 = self.kinematics.tf_A03(self.joint_angpos)[0:3,3]
            # Compute jacobian matrix
            J = self.kinematics.compute_jacobian(self.joint_angpos)
            # pseudoinverse jacobian
            pinvJ = pinv(J)


            """
            INSERT YOUR MAIN CODE HERE
            self.joint_angvel[0] = ...
            """

            '''
            Solution for the First Task
            '''

            # Defining the trajectory parameters

            # Period
            T = 4.0
            Th = T/2
            # Amplitude (from -0.2 to 0.2). A bit higher number to deal with some small errors in the motion
            A = 0.428
            # Umax of the middle phase of motion (with constant velocity)
            U = (3*A)/(2*Th)
            # Max velocity for the initial motion from 0.0 to -0.2
            U2 = U/2
            # In order for the motion to be periodical :
            my_t = t_now+0.0000000000000000001
            my_t2 = ((t_now-Th) % T)+0.0000000000000000001

            # Defining the trajectory for the velocity with respect to the report of the project
            if t_now>=Th:
                if my_t2<=Th/3:
                    y1_dot_desired = ((10*U)/((((Th)/(3)))**(3)))* my_t2**(3)-((15*U)/((((Th)/(3)))**(4))) *my_t2**(4)+6 *((U)/((((Th)/(3)))**(5))) *my_t2**(5)
                elif (Th/3)<my_t2<=(2*Th/3):
                    y1_dot_desired = U
                elif (2*Th/3)<my_t2<=Th:
                    y1_dot_desired = ((10*U)/((((Th)/(3)))**(3)))* (-my_t2+Th)**(3)-((15*U)/((((Th)/(3)))**(4))) *(-my_t2+Th)**(4)+6 *((U)/((((Th)/(3)))**(5))) *(-my_t2+Th)**(5)
                elif Th<my_t2<=(4*Th/3):
                    y1_dot_desired =-( ((10*U)/((((Th)/(3)))**(3)))* (my_t2-Th)**(3)-((15*U)/((((Th)/(3)))**(4))) *(my_t2-Th)**(4)+6 *((U)/((((Th)/(3)))**(5))) *(my_t2-Th)**(5))
                elif (4*Th/3)<my_t2<=(5*Th/3):
                    y1_dot_desired = -U
                elif (5*Th/3)<my_t2<=(2*Th):
                    y1_dot_desired = -(((10*U)/((((Th)/(3)))**(3)))* (-(my_t2-Th)+Th)**(3)-((15*U)/((((Th)/(3)))**(4))) *(-(my_t2-Th)+Th)**(4)+6 *((U)/((((Th)/(3)))**(5))) *(-(my_t2-Th)+Th)**(5))

            elif t_now<Th:
                if 0<my_t<=(Th/3):
                    y1_dot_desired =-( ((10*U2)/((((Th)/(3)))**(3)))* (my_t)**(3)-((15*U2)/((((Th)/(3)))**(4))) *(my_t)**(4)+6 *((U2)/((((Th)/(3)))**(5))) *(my_t)**(5))
                elif (Th/3)<my_t<=(2*Th/3):
                    y1_dot_desired = -U2
                elif (2*Th/3)<my_t<=(Th):
                    y1_dot_desired = -(((10*U2)/((((Th)/(3)))**(3)))* (-(my_t)+Th)**(3)-((15*U2)/((((Th)/(3)))**(4))) *(-(my_t)+Th)**(4)+6 *((U2)/((((Th)/(3)))**(5))) *(-(my_t)+Th)**(5))


            current_x = p_0E[0,0]
            current_y = p_0E[1,0]
            current_z = p_0E[2,0]
            # Proportional Controller for the motion across x and z dimensions
            Kpx = 0.8
            Kpz = 0.8
            error_x = 0.616809-current_x
            x1_dot_desired = Kpx*(error_x)

            error_z = 0.1998901-current_z
            z1_dot_desired = Kpz*(error_z)

            p1_dot_desired = np.transpose(np.matrix( [x1_dot_desired, y1_dot_desired , z1_dot_desired] ))
            # the q_dot1 for the first task
            task1 = np.dot(pinvJ, p1_dot_desired)


            '''
            Solution for the Second Task
            '''

            # Defining the parameters of the robot
            l1 = 0.267
            l2 = 0.293
            l3 = 0.0525
            l4 = 0.3512
            l5 = 0.1232
            c1 = np.cos(self.joint_angpos[0])
            s1 = np.sin(self.joint_angpos[0])
            c2 = np.cos(self.joint_angpos[1])
            s2 = np.sin(self.joint_angpos[1])
            c3 = np.cos(self.joint_angpos[2])
            s3 = np.sin(self.joint_angpos[2])
            c4 = np.cos(self.joint_angpos[3])
            s4 = np.sin(self.joint_angpos[3])
            sin1 = np.sin(0.2225)
            cos1 = np.cos(0.2225)

            # The position between the two obstacles
            # We want to minimize the distance between some joints and this position
            obstacleMed = (obstacle1[1]+1.35*obstacle2[1])/2

            # gains kc
            kc1, kc2, kc3 = [35.0, 800.0, 0.0]
            # the criterion function
            V = kc1*(P0_3[1,0]-obstacleMed)**2 + kc2*(P0_4[1,0]-obstacleMed)**2 + kc3*(P0_5[1,0]-obstacleMed)**2
            # the gradient vectors that are computed in the report
            gradient_vector1 = 2*kc1*(P0_3[1,0]-obstacleMed)*l2*c1*s2 + 2*kc2*(P0_4[1,0]-obstacleMed)*(l3*c1*c2*c2-l3*s1*s3+l2*c1*s2)+2*kc3*(P0_5[1,0]-obstacleMed)*(l4*sin1*(c1*c2*c3*c4-s1*s3*c4+c1*s2*s4)-l4*cos1*(-c1*c2*c3*s4+s1*s3*s4+c1*s2*c4)+l3*(c1*c2*c3-s1*s3)+l2*c1*s2)
            gradient_vector2 = 2*kc1*(P0_3[1,0]-obstacleMed)*l2*s1*c2 + 2*kc2*(P0_4[1,0]-obstacleMed)*(-l3*s1*s2*c3+l2*s1*c2)+2*kc3*(P0_5[1,0]-obstacleMed)*(l4*sin1*(-s1*s2*c3*c4+s1*c2*s4)-l4*cos1*(s1*s2*c3*s4+s1*c2*c4)+l3*(-s1*s2*c3)+l2*s1*c2)
            gradient_vector3 = 2*kc2*(P0_4[1,0]-obstacleMed)*(-l3*s1*c2*s3+l3*c1*c3)+2*kc3*(P0_5[1,0]-obstacleMed)*(l4*sin1*(-s1*c2*s3*c4+c1*c3*c4)-l4*cos1*(s1*c2*s3*s4-c1*c3*s4)+l3*(-s1*c2*s3+c1*c3))
            gradient_vector4 = 2*kc3*(P0_5[1,0]-obstacleMed)*(l4*sin1*(-s1*c2*c3*s4+s1*s2*c4-c1*s3*s4)-l4*cos1*(-s1*c2*c3*c4-c1*s3*c4-s1*s2*s4))
            gradient_vector5 = 0.0
            gradient_vector6 = 0.0
            gradient_vector7 = 0.0
            gradient_vector=-1*np.transpose(np.matrix([gradient_vector1,gradient_vector2,gradient_vector3,gradient_vector4,gradient_vector5,gradient_vector6,gradient_vector7]))

            I7 = np.identity(7)
            # the q_dot2 for the second task
            task2 = np.dot((np.subtract(I7, np.dot(pinvJ,J))),gradient_vector)
            # The total solution
            ang_vel_desired = np.add(task1, task2)
            # the desired angular velocities of the joints
            for i in range(0,7):
                self.joint_angvel[i] = ang_vel_desired[i,0]




            '''
            Plots
            [ This segment is commented out because it causes the simulation to freeze while creating the plots and messes up the simulation :( ]
            '''

            '''
            # Some parameters used for the plots
            x1_dot_desired = 0.0
            z1_dot_desired = 0.0
            velocity_x = (current_x - prev_x)/(t_now-t_prev)
            velocity_y = (current_y - prev_y)/(t_now-t_prev)
            velocity_z = (current_z - prev_z)/(t_now-t_prev)
            current_velocity = np.sqrt((velocity_x)**2+(velocity_y)**2+(velocity_z)**2)
            ideal_velocity = np.sqrt((x1_dot_desired)**2+(y1_dot_desired)**2+(z1_dot_desired)**2)
            velocity_error = ideal_velocity - current_velocity

            # Keeping some info for time=2*T in order to make plots
            if t_now<=10.0 and t_now>=2.0:
                # distances from obstacles
                distanceObs1_Joint4.append(np.sqrt((P0_4[1,0]-y0)**2))
                distanceObs2_Joint4.append(np.sqrt((P0_4[1,0]-y1)**2))
                distanceObs1_Joint5.append(np.sqrt((P0_5[1,0]-y0)**2))
                distanceObs2_Joint5.append(np.sqrt((P0_5[1,0]-y1)**2))
                distanceObs1_Joint3.append(np.sqrt((P0_3[1,0]-y0)**2))
                distanceObs2_Joint3.append(np.sqrt((P0_3[1,0]-y1)**2))
                # times axis
                t_now_points.append(t_now)
                velocity_error_array.append(velocity_error)
                velocity_array.append(current_velocity)
                desired_velocity_array.append(ideal_velocity)
                # trajectory
                x_pos.append(current_x)
                y_pos.append(current_y)
                z_pos.append(current_z)
                # qi and qi_dot
                for i in range(7):
                    joints_positions[i].append(self.joint_angpos[i])
                    joints_velocities[i].append((self.joint_angpos[i]-joint_angpos_prev[i])/(t_now-t_prev))

            # When all necessary info is kept, we create the plots
            if t_now>=10.0 and done:
                import matplotlib.pyplot as plt
                # plots created
                done = False

                plt.figure(1)
                plt.xlabel('time')
                plt.ylabel('Distance of Joint 4 From Obstacles')
                plt.plot(t_now_points, distanceObs1_Joint4)
                plt.plot(t_now_points, distanceObs2_Joint4)
                plt.legend(['Obstacle 1 (y=-0.2)','Obstacle 2 (y=+0.2)'])
                plt.savefig('distanceJoint4FromObstaclesAccrossY.png')

                plt.figure(3)
                plt.xlabel('time')
                plt.ylabel('Distance of Joint 5 From Obstacles')
                plt.plot(t_now_points, distanceObs1_Joint5)
                plt.plot(t_now_points, distanceObs2_Joint5)
                plt.legend(['Obstacle 1 (y=-0.2)','Obstacle 2 (y=+0.2)'])
                plt.savefig('distanceJoint5FromObstaclesAccrossY.png')

                plt.figure(4)
                plt.xlabel('time')
                plt.ylabel('Distance of Joint 3 From Obstacles')
                plt.plot(t_now_points, distanceObs1_Joint3)
                plt.plot(t_now_points, distanceObs2_Joint3)
                plt.legend(['Obstacle 1 (y=-0.2)','Obstacle 2 (y=+0.2)'])
                plt.savefig('distanceJoint3FromObstaclesAccrossY.png')

                plt.figure(2)
                plt.xlabel('time')
                plt.ylabel('Velocity Error')
                plt.plot(t_now_points, velocity_error_array)
                plt.savefig('VelocityError.png')

                plt.figure(25)
                plt.xlabel('time')
                plt.ylabel('X_Position')
                plt.plot(t_now_points, x_pos)
                plt.savefig('Xposition.png')

                plt.figure(26)
                plt.xlabel('time')
                plt.ylabel('Y_Position')
                plt.plot(t_now_points, y_pos)
                plt.savefig('Yposition.png')

                plt.figure(27)
                plt.xlabel('time')
                plt.ylabel('Z_Position')
                plt.plot(t_now_points, z_pos)
                plt.savefig('Zposition.png')

                plt.figure(42)
                plt.xlabel('time')
                plt.ylabel('Velocity')
                plt.plot(t_now_points, velocity_array)
                plt.plot(t_now_points, desired_velocity_array)
                plt.legend(['Velocity of End Effector','Desired Velocity of End Effector'])
                plt.savefig('RealVsIdealVelocity.png')

                plt.figure(17)
                plt.figure(figsize=(12, 8))  # Increase the figure size to 12x8 inches
                for i in range(7):
                    plt.subplot(2, 4, i+1)
                    plt.plot(t_now_points, joints_positions[i])
                    plt.xlabel('time')
                    plt.ylabel('q'+str(i+1))
                plt.tight_layout()  # Adjust subplot layout to avoid overlapping
                plt.savefig('JointPositions.png')

                plt.figure(18)
                plt.figure(figsize=(12, 8))  # Increase the figure size to 12x8 inches
                for i in range(7):
                    plt.subplot(2, 4, i+1)
                    plt.plot(t_now_points, joints_velocities[i])
                    plt.xlabel('time')
                    plt.ylabel('q'+str(i+1)+'_dot')
                plt.tight_layout()  # Adjust subplot layout to avoid overlapping
                plt.savefig('JointVelocities.png')

            # keeping some useful values from the current iteration
            t_prev = t_now
            joint_angpos_prev = self.joint_angpos
            prev_x = current_x
            prev_y = current_y
            prev_z = current_z
            '''

            # Convertion to angular position after integrating the angular speed in time
            # Calculate time interval
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9
            # Integration
            self.joint_angpos = np.add( self.joint_angpos, [index * dt for index in self.joint_angvel] )
            # Publish the new joint's angular positions
            self.joint1_pos_pub.publish(self.joint_angpos[0])
            self.joint2_pos_pub.publish(self.joint_angpos[1])
            self.joint3_pos_pub.publish(self.joint_angpos[2])
            self.joint4_pos_pub.publish(self.joint_angpos[3])
            self.joint5_pos_pub.publish(self.joint_angpos[4])
            self.joint6_pos_pub.publish(self.joint_angpos[5])
            self.joint7_pos_pub.publish(self.joint_angpos[6])

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass
