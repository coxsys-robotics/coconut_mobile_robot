#!/usr/bin/env python

# import rospy
# import roslaunch
# from geometry_msgs.msg import Pose
# from std_msgs.msg import String


# class pure_pose_reset():
# 	def __init__(self):
# 		self.first_time = True
# 		rospy.init_node('purePose_reset_node', anonymous=True)
# 		self.pure_reset = rospy.Publisher('request_purePose_reset', String, queue_size = 10)
# 		self.listener()
		
# 	def listener(self):
# 		rospy.Subscriber("pose_from_robot", Pose, self.odomCallback)
# 		rospy.spin()

# 	def odomCallback(self, msg):
# 		self.robot_pose = msg

# 		self.x_pose = self.robot_pose.position.x
# 		self.y_pose = self.robot_pose.position.y

# 		if self.first_time == True:
# 			self.old_x_pose = self.x_pose
# 			self.old_y_pose = self.y_pose
# 			self.first_time = False

# 		diff_y = abs(self.old_y_pose - self.y_pose)
# 		diff_x = abs(self.old_x_pose - self.x_pose)

# 		rospy.loginfo("{},{}".format(diff_x, diff_y))

# 		if diff_x >= 2 or diff_y >= 2:
# 			self.pure_reset.publish("reset")
# 			self.first_time = True


# 		self.old_x_pose = self.x_pose
# 		self.old_y_pose = self.y_pose


# if __name__ == '__main__':
# 	try:
# 		process = pure_pose_reset()
# 	finally:
# 		rospy.signal_shutdown("Exit")

import rospy
import tf
import roslaunch


if __name__ == '__main__':
    rospy.init_node('purePose_reset_node', disable_signals=True)

    listener = tf.TransformListener()
    first_time = True
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        x_pose = trans[0]
        y_pose = trans[1]

        
        if first_time == True:
            old_x_pose = x_pose
            old_y_pose = y_pose
            first_time = False
            continue

        diff_y = abs(old_y_pose-y_pose)
        diff_x = abs(old_x_pose-x_pose)

        rospy.loginfo("{},{}".format(diff_x, diff_y))

        if diff_x > 2 or diff_y > 2:
            rotate360_node = roslaunch.core.Node(package='coconut_deli_sensor', 
												 node_type='rotateBy_odom.py',
												 name='rotateBy_odom_node')

            rotate360_node.args = "_rotate_target:=%d" %360

            rotate360_launch = roslaunch.scriptapi.ROSLaunch()
            rotate360_launch.start()

            rotate360_process = rotate360_launch.launch(rotate360_node)

            while rotate360_process.is_alive():
                if rotate360_process.is_alive() == False:
                    break

            rotate360_process.stop()
            first_time = True
        
        old_x_pose = x_pose
        old_y_pose = y_pose

        rate.sleep()

rospy.signal_shutdown("Exit")