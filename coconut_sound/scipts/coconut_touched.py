#!/usr/bin/env python

# Import necessary package 
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from os.path import expanduser

# Initialize home directory
home = expanduser("~")

# Class for playing desired sound when someone touching the robot touch screen
class play_touch:

	# Initial state
	def __init__(self):
		rospy.init_node('coconut_touched_node')
		self.touchhandle = SoundClient()
		self.touch_volume = 1.0
		self.listener()

	# Call play function when subscribe to 'touch_sound' topic in string type
	def listener(self):
		rospy.Subscriber('touch_sound', String, self.play)
		rospy.spin()
	
	# Play desired sound to the user depend on which button have pushed by user
	def play(self, user_input):
		if user_input.data == "engage_process":
			self.touchhandle.playWave("{}/coconut_ws/src/UVC/coconut_sound/sound/engage_pressed.wav".format(home), self.touch_volume)
			rospy.sleep(5.0)
		elif user_input.data == "finish_process":
			self.touchhandle.playWave("{}/coconut_ws/src/UVC/coconut_sound/sound/finish_pressed.wav".format(home), self.touch_volume)
			rospy.sleep(2.0)
		elif user_input.data == "wrong_process":
			self.touchhandle.playWave("{}/coconut_ws/src/UVC/coconut_sound/sound/wrong_clicked.wav".format(home), self.touch_volume)
			rospy.sleep(4.0)


if __name__ == '__main__':
	# Call play_touch class
	process = play_touch()