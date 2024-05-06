#!/usr/bin/env python3

"""
Start ROS node to publish linear and angular velocities to mymobibot in order to perform wall following.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class mymobibot_follower():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # linear and angular velocity
        self.velocity = Twist()
        # joints' states
        self.joint_states = JointState()
        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('/mymobibot/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])

    def imu_callback(self, msg):
        # ROS callback to get the /imu

        self.imu = msg
        # (e.g. the orientation of the robot wrt the global frome is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frome is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frome is stored in :: self.imu.linear_acceleration)

        #quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #(roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)
        (roll, pitch, self.imu_yaw) = quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def sonar_front_callback(self, msg):
        # ROS callback to get the /sensor/sonar_F

        self.sonar_F = msg
        # (e.g. the distance from sonar_front to an obstacle is stored in :: self.sonar_F.range)

    def sonar_frontleft_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FL

        self.sonar_FL = msg
        # (e.g. the distance from sonar_frontleft to an obstacle is stored in :: self.sonar_FL.range)

    def sonar_frontright_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FR

        self.sonar_FR = msg
        # (e.g. the distance from sonar_frontright to an obstacle is stored in :: self.sonar_FR.range)

    def sonar_left_callback(self, msg):
        # ROS callback to get the /sensor/sonar_L

        self.sonar_L = msg
        # (e.g. the distance from sonar_left to an obstacle is stored in :: self.sonar_L.range)

    def sonar_right_callback(self, msg):
        # ROS callback to get the /sensor/sonar_R

        self.sonar_R = msg
        # (e.g. the distance from sonar_right to an obstacle is stored in :: self.sonar_R.range)

    def publish(self):

        # set configuration
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        # computing the starting time of the simulation in seconds
        time_start=rostime_now.secs + rostime_now.nsecs/1000000000
        # boolean variables in order to distinguish the 3 motion modes
        initial_mode = True
        rotation_mode = False
        main_motion_mode = False
        # the desired distance from the walls
        left_desired = 0.3
        # parameters of the PD controller
        Kp = 15.0
        Kd = 5.0
        # used for plots and for the derivation of the error for the controller :
        previous_left_error = 0.0
        t_prev = time_now
        done = True

        # used in order to save info for the plots
        linear_velocities = []
        angular_velocities = []
        t_now_points=[]
        sonar_left_errors = []
        alignment_errors = []

        while not rospy.is_shutdown():
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
             # computing the current time of the simulation in seconds
            time_sec = (rostime_now.secs + rostime_now.nsecs/1000000000)-time_start

            # keep the current measurement of the sonars
            sonar_front = self.sonar_F.range # and so on...
            sonar_left= self.sonar_L.range
            sonar_front_left= self.sonar_FL.range
            sonar_front_right= self.sonar_FR.range


            """
            INSERT YOUR MAIN CODE HERE
            self.velocity.linear.x = ...
            self.velocity.angular.z = ...
            """

            '''
            # used for debugging

            message=""
            if rotation_mode :
                message="Rotation Mode"
            elif main_motion_mode:
                message="Main Motion Mode"
            print(message)
            '''

            # Implementation of the initial mode as it is described in the report
            if initial_mode:
                self.velocity.linear.x = 0.5
                self.velocity.angular.z = 0.0
                # when finished, turn to rotation mode and make initial_mode false
                if sonar_front*np.sin(0.721240405)<0.3 :
                    initial_mode=False
                    rotation_mode = True

             # Implementation of the rotation mode as it is described in the report
            if rotation_mode:
                self.velocity.linear.x = 0.05
                self.velocity.angular.z = np.pi
                # alignment condition in order to turn to main motion mode and leave rotation mode
                if str(sonar_left+0.018)[:3]==str(sonar_front_left*np.sin(np.pi/4 + 0.721240405))[:3]:
                    rotation_mode = False
                    main_motion_mode = True

            # Implementation of the main motion mode and of the controller as it is described in the report
            if main_motion_mode:
                self.velocity.linear.x = 0.4
                left_error = left_desired-sonar_left + (sonar_left+0.018-sonar_front_left*np.sin(np.pi/4 + 0.721240405))
                left_error_dot = (left_error-previous_left_error)/(time_now-t_prev)
                self.velocity.angular.z = Kp*left_error + Kd*left_error_dot

                # keeping some useful values for the derivation error_dot
                previous_left_error = left_error
                t_prev = time_now

                # condition in order to turn to rotation mode if needed
                if sonar_front<0.39 or sonar_front_right<0.44:
                    main_motion_mode = False
                    rotation_mode = True


            '''
            Plots
            [ This segment is commented out because it causes the simulation to freeze while creating the plots and messes up the simulation :( ]
            '''

            '''
            # Keeping some info for time=43 seconds (full map rotation) in order to make the plots
            if time_sec>=0.0 and time_sec<=43.0:
                # saving velocities and errors
                linear_velocities.append(self.velocity.linear.x)
                angular_velocities.append(self.velocity.angular.z)
                sonar_left_errors.append(0.3-sonar_left)
                alignment_errors.append((sonar_left+0.018-sonar_front_left*np.sin(np.pi/4 + 0.721240405)))
                # times axis
                t_now_points.append(time_sec)

            # When all necessary info is kept, we create the plots
            path = '/home/dimitris/Desktop/NTUA_Ubuntu/Robotics/Project/Project2A/images/'
            if time_sec>=43.0 and done:
                import matplotlib.pyplot as plt
                # plots created
                done = False
                plt.figure(1)
                plt.xlabel('time')
                plt.ylabel('Linear Velocity Control Signal')
                plt.plot(t_now_points, linear_velocities)
                plt.savefig(path+'LinearVelocity.png')

                plt.figure(2)
                plt.xlabel('time')
                plt.ylabel('Angular Velocity Control Signal')
                plt.plot(t_now_points, angular_velocities)
                plt.savefig(path+'AngularVelocity.png')

                plt.figure(3)
                plt.xlabel('time')
                plt.ylabel('Sonar Left Errors')
                plt.plot(t_now_points, sonar_left_errors)
                plt.savefig(path+'SonarLeftErrors.png')

                plt.figure(4)
                plt.xlabel('time')
                plt.ylabel('Alignment Errors')
                plt.plot(t_now_points, alignment_errors)
                plt.savefig(path+'AlignmentErrors.png')
            '''

            # Calculate time interval (in case is needed)
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9

            # Publish the new joint's angular positions
            self.velocity_pub.publish(self.velocity)

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    follower = mymobibot_follower(rate)
    rospy.on_shutdown(follower.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException:
        pass
