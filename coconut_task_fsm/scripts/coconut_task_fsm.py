#!/usr/bin/env python3

# Import necessary package 
import rospy
import smach
import smach_ros
import roslaunch
from read_roomFile import read_room
from read_actionFile import read_action
from clear_costMap import clear_costmaps
from switch_obstacleAvoidance import open_obstacleAvoidance, close_obstacleAvoidance

room_data = read_room() 


# Class for checking robot position
class localize_robot_position(smach.State):
	"""
	after receive task command, robot rotate in place to self-localize.
	(will work only if localization algorithm allow robot position to jump)
	"""

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['localized'],
							input_keys=['roomList_input'],
							output_keys=['roomList_output'])

	# Execution function
	def execute(self, userdata):
		# print('\n')
		rospy.loginfo('Executing state localize robot position')

		# # Robot rotate around itself
		rospy.loginfo('Robot rotate around for localize the real position')
		localize_node = roslaunch.core.Node(package='coconut_sensor', 
											node_type='rotateBy_odom.py', 
											name='rotateBy_odom_node',
											output="screen")
		localize_node.args = "_rotate_target:=%d" %(360)
		localize_launch = roslaunch.scriptapi.ROSLaunch()
		localize_launch.start()
		localize_process = localize_launch.launch(localize_node)
		while localize_process.is_alive():
			if localize_process.is_alive() == False:
				break
		localize_process.stop()

		rospy.loginfo('Robot localized itself')
		
		# Output equal to input
		userdata.roomList_output = userdata.roomList_input
		return 'localized'


# Class for moving the robot to the room
class move2room(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['move_success'],
							input_keys=['roomList_input'],
							output_keys=['roomList_output'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state move to room')
		rospy.loginfo('Robot have been moving to {}'.format(userdata.roomList_input[0]))

		# # Robot moving to the next goal and stop when detected the obstacle
		robot_goal = room_data[userdata.roomList_input[0]]

		# open_obstacleAvoidance()
		nav_node = roslaunch.core.Node(package='coconut_navigation', 
									   node_type='send_goal.py', 
									   name='movebase_client_py')
		nav_node.args = """ _position_x:={}
							_position_y:={}
							_position_z:={}
							_orientation_x:={} 
							_orientation_y:={} 
							_orientation_z:={}  
							_orientation_w:={} """\
							.format(robot_goal[0],
									robot_goal[1],
									0.0,
									0.0,
									0.0,
									robot_goal[2],
									robot_goal[3])
		nav_launch = roslaunch.scriptapi.ROSLaunch()
		nav_launch.start()
		nav_process = nav_launch.launch(nav_node)
		while nav_process.is_alive():
			if nav_process.is_alive == False:
				break
		nav_process.stop()
		# close_obstacleAvoidance()

		# Clear cost map
		clear_costmaps()

		# Output equal to input
		userdata.roomList_output = userdata.roomList_input
		return 'move_success'


# Class for verify that robot reached to room or not
class reach2room(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['reach_success'],
							input_keys=['roomList_input'],
							output_keys=['roomList_output'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state reach to room')
		rospy.loginfo('Robot reach to {}'.format(userdata.roomList_input[0]))
		
		# FEED BACK TO NURSE ################################################

		# Output equal to input
		userdata.roomList_output = userdata.roomList_input
		return 'reach_success'


# Class for aligning the robot to each room
class roomAlignment(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self,
							outcomes=['align_success'],
							input_keys=['roomList_input'],
							output_keys=['roomList_output'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state room alignment')
		rospy.loginfo('Robot align to {}'.format(userdata.roomList_input[0]))
		robot_goal = room_data[userdata.roomList_input[0]]
		# # Robot start aligning to current room
		read_action([robot_goal[0], robot_goal[1], robot_goal[2], robot_goal[3]], "turn2currentRoom")
		rospy.loginfo("Robot start align with {} by {} degree".format(userdata.roomList_input[0],read_action.rotate2currentRoom))

		roomAlignment_node = roslaunch.core.Node(package='coconut_sensor', 
												node_type='rotateBy_odom.py', 
												name='rotateBy_odom_node',
												output="screen")
		roomAlignment_node.args = "_rotate_target:=%d" %read_action.rotate2currentRoom
		roomAlignment_launch = roslaunch.scriptapi.ROSLaunch()
		roomAlignment_launch.start()
		roomAlignment_process = roomAlignment_launch.launch(roomAlignment_node)
		while roomAlignment_process.is_alive():
			if roomAlignment_process.is_alive() == False:
				break
		roomAlignment_process.stop()

		# Output equal to input
		userdata.roomList_output = userdata.roomList_input
		return 'align_success'


# Class for waiting user to pick up food
class wait4user(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['task_success','all_tasks_success'],
							input_keys=['roomList_input'],
							output_keys=['roomList_output'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state wait for user')

		if len(userdata.roomList_input) == 1 and userdata.roomList_input == "base_station":
			return 'all_tasks_success'

		# Delete reached goal
		del userdata.roomList_input[0]

		# Output equal to input
		userdata.roomList_output = userdata.roomList_input
		if len(userdata.roomList_input) > 0:
			return 'task_success'
		elif len(userdata.roomList_input) == 0:
			return 'all_tasks_success'


# Class for waiting nurse command again
class wait4nextround(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['finished_process'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state wait for user')
		return 'finished_process'


# Finite state machine main loop
def main():
	# room_list = rospy.get_param("/coconut_fsm_subscriber_node/room", None)
	room_list = rospy.get_param("/rooms_to_deliver", ["1001", "1002", "1003", "1004"])
	room_list.append("base_station")
	print(room_list)
	sm_nav = smach.StateMachine(outcomes=['shutdown'])
	sm_nav.userdata.room_list = room_list
	with sm_nav:
		smach.StateMachine.add('LOCALIZE_ROBOT', localize_robot_position(),
								transitions={'localized':'MOVE2ROOM'},
								remapping={ 'roomList_input':'room_list',
											'roomList_output':'room_list'})

		smach.StateMachine.add('MOVE2ROOM', move2room(),
								transitions={'move_success':'SM_ACTION'},
								remapping={ 'roomList_input':'room_list',
											'roomList_output':'room_list'})

		smach.StateMachine.add('WAIT4NEXTROUND', wait4nextround(),
								transitions={'finished_process':'shutdown'})


		sm_act = smach.StateMachine(outcomes=['deli_finished', 'all_finished'])
		sm_act.userdata.room_list = room_list
		with sm_act:
			smach.StateMachine.add('REACH2ROOM', reach2room(), 
									transitions={'reach_success':'ROOM_ALIGNMENT'},
									remapping={ 'roomList_input':'room_list',
												'roomList_output':'room_list'})

			smach.StateMachine.add('ROOM_ALIGNMENT', roomAlignment(),
									transitions={'align_success':'WAIT4USER'},
									remapping={ 'roomList_input':'room_list',
												'roomList_output':'room_list'})
			
			smach.StateMachine.add('WAIT4USER', wait4user(),
									transitions={'task_success':'deli_finished', 'all_tasks_success':'all_finished'},
									remapping={ 'roomList_input':'room_list',
												'roomList_output':'room_list'}) 

		smach.StateMachine.add('SM_ACTION', sm_act,
							   transitions={'deli_finished':'MOVE2ROOM','all_finished':'WAIT4NEXTROUND'})


	sis = smach_ros.IntrospectionServer('server_name', sm_nav, 'SM_NAV/SM_ACT')
	sis.start()
	outcome = sm_nav.execute()
	# rospy.spin()
	sis.stop()


if __name__ == '__main__':
	# Define node name
	rospy.init_node('coconut_fsm_node', anonymous=False)
	main()
