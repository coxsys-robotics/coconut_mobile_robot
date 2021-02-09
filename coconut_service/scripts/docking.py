#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist

from nav_msgs.msg import Odometry

"""
Phase 0: Rotate until marker is found
Phase 1: Loop 
        - Determine marker_pitch and x_position
        - Rotate until perpendicular to marker
        - move until center of robot is on/near marker yz plane
        - Rotate back to face with marker
        - if marker x_position low enough procedd to next phase, if not just loop over phase 1 again
Phase 2: Approach the marker instead of running away because robot cannot dock unless it come closer.   
"""

class docking_motion_control:
    """
    unit: meter, radian
    """
    def __init__(self, pose_topic="aruco_pose", x_offset=0.03, z_offset=0.2, tolerance_pitch=0.1, vel_topic="nav_vel",
                max_linear_vel=0.1, max_angular_vel=0.1):
        ### marker position. don't need to care for y position
        self.pose_topic_name = pose_topic
        self.pose_sub = rospy.Subscriber(self.pose_topic_name, PoseStamped, self.marker_pose_callback)

        self.x_offset = x_offset
        self.z_offset = z_offset
        self.tolerance_pitch = tolerance_pitch
        self.get_new_pose = False

        self.marker_pose_x = 0
        self.marker_pose_z = 0
        self.marker_pose_x_in_image = -1
        self.prv_marker_pose_x_in_image = -1
        self.marker_pitch = 0 ## compute from quaternion. WRT camera frame.

        ### self position and orientation
        self.self_pose_topic = "t265/odom/sample"
        self.self_pose_sub = rospy.Subscriber(self.self_pose_topic, Odometry, self.self_pose_callback)
        self.self_x = 0
        self.self_y = 0
        self.self_yaw = 0
        self.prv_self_yaw = 0 ## previous self yaw

        ### velocity command
        self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=10)
        self.vel_msg = Twist()
        self.linear_vel = 0
        self.angular_vel = 0
        self.max_linear = max_linear_vel
        self.max_angular = max_angular_vel

        ### docking status
        self.docking_phase = 0
        self.linear_complete = False ## x and z axis value is in desired range
        self.angular_complete = False ## radian value is in desired range
        self.docking_complete = False ## True when both linear and angular are complete

        self.start_x = 0 ## robot's start position when try to align it's center with marker's.
        self.start_y = 0
        self.goal_distance = 0
        self.goal_angular = 0
        self.goal_sum = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_count = 0
        self.goal_max_count = 40

        self.align_lose_sigth = False

        self.robot_width = 0.64

    def self_pose_callback(self, data):
        w = data.pose.pose.orientation.w
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z

        t1 = 2.0*(w*z + y*x)
        t2 = 1 - (2 * ( math.pow(y,2) + math.pow(z,2)))
        self.self_yaw = math.atan2( t2, t1)

        self.self_x = data.pose.pose.position.x - ((self.robot_width/2) * math.sin(self.self_yaw))
        self.self_y = data.pose.pose.position.y - ((self.robot_width/2) * math.cos(self.self_yaw))
        

    def marker_pose_callback(self, data):
        self.get_new_pose = True
        self.marker_pose_x = data.pose.position.x
        self.marker_pose_z = data.pose.position.z
        self.marker_pose_x_in_image = data.pose.position.y ### range 0.0-1.0 from image left to right
        # self.marker_orien_w = data.pose.orientation.w
        # self.marker_orien_x = data.pose.orientation.x
        # self.marker_orien_y = data.pose.orientation.y
        # self.marker_orien_z = data.pose.orientation.z
        w = data.pose.orientation.w
        x = data.pose.orientation.x
        y = data.pose.orientation.y
        z = data.pose.orientation.z

        # t1 = 2.0*(y*z + w*x)
        # t2 = 1 - (2 * ( math.pow(y,2) + math.pow(z,2)))
        # self.marker_yaw = math.atan2( t1, t2)

        self.marker_pitch = math.asin( 2 * ( (w*y) - (x*z) ) )

    """calculate robot's goal pitch and linear distance to move."""
    def calculate_target_pitch(self):
        self.goal_angular = self.self_yaw + (abs(self.marker_pitch)/self.marker_pitch) * ((math.pi/2) - abs(self.marker_pitch))
        if(self.goal_angular >= math.pi):
            self.goal_angular -= 2*math.pi
        elif(self.goal_angular <= -math.pi):
            self.goal_angular += 2*math.pi
        # print(self.goal_angular)
        self.goal_sum += self.goal_angular
        self.goal_count+=1

    def calculate_target_xy(self):
        distance = self.marker_pose_z + (self.robot_width/2)
        self.goal_distance = distance * math.sin(abs(self.marker_pitch))
        self.goal_x = self.self_x + ((distance) * math.sin(abs(self.goal_angular)))
        self.goal_y = self.self_y + ((distance) * math.cos(abs(self.goal_angular)))
        self.start_x = self.self_x
        self.start_y = self.self_y

    def traveled_distance(self):
        return math.sqrt(math.pow(self.self_x - self.start_x, 2) + math.pow(self.self_y - self.start_y, 2))

    def reset_goal(self):
        self.goal_angular = 0
        self.goal_sum = 0
        self.goal_count = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_distance = 0

    def motion_control(self):
        ### 0: keep rotating until marker found
        if(self.docking_phase==0):
            if (not self.get_new_pose): ## not see marker yet
                if(self.prv_marker_pose_x_in_image < 0.5):
                    self.angular_vel = self.max_angular
                elif(self.prv_marker_pose_x_in_image > 0.5):
                    self.angular_vel = -self.max_angular
            else: ## when see marker, stop rotate and proceed to next phase.
                self.angular_vel = 0
                self.docking_phase = 1
        
        ### 1: turn robot's orientation to face with marker.
        elif(self.docking_phase==1):
            if(self.marker_pose_x < self.x_offset - 0.04):
                self.angular_vel = self.max_angular * min(1, abs((self.x_offset)/(self.x_offset - 0.04)))
            elif(self.marker_pose_x > self.x_offset + 0.04):
                self.angular_vel = -self.max_angular * min(1, abs((self.x_offset)/(self.x_offset - 0.04)))
            else:
                self.angular_vel = 0
                if(self.marker_pose_x < self.x_offset+0.1 and self.marker_pose_x > self.x_offset-0.1 and 
                    self.marker_pitch > self.tolerance_pitch - 0.1 and self.marker_pitch < self.tolerance_pitch + 0.1):
                    self.docking_phase = 3
                else:   
                    self.calculate_target_pitch()

            if(self.goal_count == self.goal_max_count):
                self.goal_angular = self.goal_sum / self.goal_count
                self.calculate_target_xy()
                print(self.goal_angular)
                print(self.goal_x)
                print(self.goal_y)
                print("-------------")
                if(self.goal_distance <= 0.1):
                    self.docking_phase = 3
                else:
                    self.docking_phase = 2

        ### 2: rotate to perpendicular with marker.
        elif(self.docking_phase==2):
            d = self.self_yaw - self.goal_angular
            if(d > math.pi):
                d -= 2*math.pi
            elif(d < -math.pi):
                d += 2*math.pi
            print(self.traveled_distance())
            if( d < -0.1 ):
                self.angular_vel = -self.max_angular
            elif( d > 0.1 ):
                self.angular_vel = self.max_angular
            elif( self.traveled_distance() < abs(self.goal_distance) ): #abs(self.self_x - self.goal_x) > 0.05 or abs(self.self_y - self.goal_y) > 0.05 ):
                self.angular_vel = 0
                self.linear_vel = self.max_linear
                # print("hello")
            else:
                self.reset_goal()
                self.angular_vel = 0
                self.linear_vel = 0
                self.docking_phase = 0
            

        ### 3: move straigth forward until 
        ### forward and keep marker at middle
        elif(self.docking_phase == 3):
            if(self.get_new_pose):
                ### calculate linear velocity
                if(self.marker_pose_z > self.z_offset):
                    self.linear_vel = self.max_linear 
                else:
                    self.linear_vel = 0
                    self.linear_complete = True
                ### calculate angular velocity
                if(self.marker_pose_x < self.x_offset - 0.01):
                    self.angular_vel = self.max_angular
                    self.angular_complete = False
                elif(self.marker_pose_x > self.x_offset + 0.01):
                    self.angular_vel = -self.max_angular
                    self.angular_complete = False
                else:
                    self.angular_vel = 0
                    self.angular_complete = True
            else:
                if(self.prv_marker_pose_x_in_image <= 0.5):
                    self.angular_vel = self.max_angular
                elif(self.prv_marker_pose_x_in_image > 0.5):
                    self.angular_vel = -self.max_angular
                self.linear_vel = 0
            if(self.linear_complete and self.angular_complete):
                print("*************************")
                self.docking_phase = 4

        ### align self pitch to marker
        elif(self.docking_phase == 4):
            if(self.get_new_pose):

                if(self.align_lose_sigth):
                    self.linear_complete = False
                    self.angular_complete = False
                    if(self.marker_pose_z > self.z_offset + 0.4):
                        self.align_lose_sigth = False
                        self.docking_phase = 3
                        print("---------------------")

                elif( self.marker_pitch > self.tolerance_pitch*0.3):
                    self.angular_vel = self.max_angular * 0.2
                elif (self.marker_pitch <= -self.tolerance_pitch*0.3):
                    self.angular_vel = -self.max_angular * 0.2
                elif(self.marker_pose_x > self.x_offset+0.01 or self.marker_pose_x < self.x_offset-0.01 ):
                    print("++++++++++++++++++++")
                    self.align_lose_sigth = True
                    if(self.angular_vel > 0):
                        self.angular_vel = self.max_angular
                    else:
                        self.angular_vel = -self.max_angular
                    # self.angular_vel = self.angular_vel
                    self.linear_vel = -self.max_linear *2.0
                else:
                    self.docking_complete = True
                    self.linear_vel = 0
                    self.angular_vel = 0

            else:
                print("++++++++++++++++++++")
                self.align_lose_sigth = True
                if(self.angular_vel > 0):
                    self.angular_vel = self.max_angular
                else:
                    self.angular_vel = -self.max_angular
                # self.angular_vel = self.angular_vel
                self.linear_vel = -self.max_linear *2.0

        ### publish vel command
        self.vel_msg.linear.x = self.linear_vel
        self.vel_msg.angular.z = self.angular_vel
        self.vel_pub.publish(self.vel_msg)

        self.prv_marker_pose_x_in_image = self.marker_pose_x_in_image
        self.get_new_pose = False

    def filter_pose(self):
        ### filter noise from received pose 
        pass


if(__name__ == "__main__"):
    motion_control = docking_motion_control()
    rospy.init_node("test_dock")
    while(not rospy.is_shutdown()):
        if( not motion_control.docking_complete):
            # print(motion_control.docking_phase)
            # print(motion_control.get_new_pose)
            # print(motion_control.self_x)
            # print(motion_control.self_y)
            # print("=+=+=+=+=+=+=+=+=+=+=")
            motion_control.motion_control()
            
        else:
            break
        rospy.sleep(0.1)








