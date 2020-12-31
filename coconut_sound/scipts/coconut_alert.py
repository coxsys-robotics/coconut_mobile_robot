#!/usr/bin/env python

# Import necessary package 
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from os.path import expanduser

# Initialize home directory
home = expanduser("~")

# Class for alert the alarm sound when obstacle detected by lidar
class play_alert:

	# Initial state
	def __init__(self):
		rospy.init_node('coconut_alert_node')
		self.alerthandle = SoundClient()
		self.alert_to_play = '{}/coconut_ws/src/coconut_sound/sound/obstacle.wav'.format(home)
		self.alert_cycle_time = 7.0
		self.listener()

	# Call play function when subscribe to 'alert' topic in string type
	def listener(self):
		rospy.Subscriber('alert', String, self.play)
		rospy.spin()
	
	# Alert the alarm sound to the user for give way to robot
	def play(self, data):
		rospy.sleep(1)
		self.alert_volume = rospy.get_param("/coconut_gui_node/alert_volume", 1.0)
		self.alerthandle.playWave(self.alert_to_play, self.alert_volume)
		rospy.sleep(self.alert_cycle_time)


if __name__ == '__main__':
	# Call play_alert class
	process = play_alert()