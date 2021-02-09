#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Byte
import numpy as np


class State2D:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0


"""
calculate robot odometry from robot's feedback velocity.
rk4 is a method to filter noise in linear position.
TODO: Find way to filter theta as well. Make wheel odom more accurate.
"""
class RK4Odometry:
    def __init__(self):
        rospy.loginfo("Starting RK4 Odometry builder.")
        rospy.Subscriber("/feedback_vel", Twist, self.feedback_callback)
        rospy.Subscriber("/ult_data", Byte, self.ult_callback)

        ### TODO: Change topic "/debug_val" to topic of push button
        rospy.Subscriber("/debug_val", Twist, self.emer_callback)
        ###

        rospy.Subscriber("/t265/odom/sample", Odometry, self.t265_callback)
        self.t265_odom = Odometry()

        self.odom_pub = rospy.Publisher("/wheel_odometry", Odometry, queue_size=5)
        self.freq = rospy.get_param("~freq", default=50)
        self.Dt = 1.0 / self.freq
        self.state = State2D()
        self.odom = Odometry()
        self.odom.header.frame_id = "odom_rk4"
        self.odom.child_frame_id = "base_footprint"
        self.odom.pose.covariance = [1e-5, 0, 0, 0, 0, 0,
                                     0, 1e-5, 0, 0, 0, 0,
                                     0, 0, 1e12, 0, 0, 0,
                                     0, 0, 0, 1e12, 0, 0,
                                     0, 0, 0, 0, 1e12, 0,
                                     0, 0, 0, 0, 0, 1e-3]
        self.odom.twist.covariance = [0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0]

        self.emer_pressed = False

    def ult_callback(self, data):
        data_bin = "{:08b}".format(data.data)
        drive_state = data_bin[3]
        if drive_state == "0":
            self.odom.pose.covariance = [1e12, 0, 0, 0, 0, 0,
                                        0, 1e12, 0, 0, 0, 0,
                                        0, 0, 1e12, 0, 0, 0,
                                        0, 0, 0, 1e12, 0, 0,
                                        0, 0, 0, 0, 1e12, 0,
                                        0, 0, 0, 0, 0, 1e12]
            self.odom.twist.covariance = [1e12, 0, 0, 0, 0, 0,
                                        0, 1e12, 0, 0, 0, 0,
                                        0, 0, 1e12, 0, 0, 0,
                                        0, 0, 0, 1e12, 0, 0,
                                        0, 0, 0, 0, 1e12, 0,
                                        0, 0, 0, 0, 0, 1e12]
        else:
            self.odom.pose.covariance = [1e-5, 0, 0, 0, 0, 0,
                                        0, 1e-5, 0, 0, 0, 0,
                                        0, 0, 1e12, 0, 0, 0,
                                        0, 0, 0, 1e12, 0, 0,
                                        0, 0, 0, 0, 1e12, 0,
                                        0, 0, 0, 0, 0, 1e-3]
            self.odom.twist.covariance = [0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0]

    def feedback_callback(self, data):
        new_s = self.rk4_integrate(data.linear.x, data.angular.z)
        self.state.x = new_s[0]
        self.state.y = new_s[1]
        self.state.theta = new_s[2]
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x = self.state.x
        self.odom.pose.pose.position.y = self.state.y
        q = quaternion_from_euler(0, 0, self.state.theta)
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]

        self.odom.twist.twist.linear.x = data.linear.x
        self.odom.twist.twist.angular.z = data.angular.z

        self.odom_pub.publish(self.odom)

    def kinematics(self, s, u):
        """
        x = s[0]
        y = s[1]
        theta = s[2]

        v = u[0]
        w = u[1]
        """

        d_x = np.cos(s[2]) * u[0]
        d_y = np.sin(s[2]) * u[0]
        d_theta = u[1]
        return np.array([d_x, d_y, d_theta])

    ### 
    def rk4_integrate(self, v, w):
        s = np.array([self.state.x, self.state.y, self.state.theta])
        u = np.array([v, w])
        k1 = self.kinematics(s, u)
        k2 = self.kinematics(s + k1 * 0.5 * self.Dt, u)
        k3 = self.kinematics(s + k2 * 0.5 * self.Dt, u)
        k4 = self.kinematics(s + k3 * self.Dt, u)
        d_s = (k1 + (2.0 * k2) + (2.0 * k3) + k4) / 6.0
        new_s = s + d_s * self.Dt
        return new_s

    def emer_callback(self,data):
        if(data.linear.x==0 or data.linear.x==3 and not self.emer_pressed):
            self.emer_pressed = True
        elif(data.linear.x==1 and self.emer_pressed):
            self.emer_pressed = False
            self.state.x = self.t265_odom.pose.pose.position.x
            self.state.y = self.t265_odom.pose.pose.position.y
            q = [self.t265_odom.pose.pose.orientation.x, self.t265_odom.pose.pose.orientation.y, 
                self.t265_odom.pose.pose.orientation.z, self.t265_odom.pose.pose.orientation.w]
            self.state.theta = euler_from_quaternion(q)[2]
            
    """
    if emer button is pressed, there will be no feedback_vel so use t265 odom instead.
    """
    def t265_callback(self,data):
        if(self.emer_pressed):
            self.t265_odom = data

if __name__ == "__main__":
    rospy.init_node("ros_node")
    rk4_odom = RK4Odometry()
    rospy.spin()
