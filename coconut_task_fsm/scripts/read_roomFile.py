#!/usr/bin/env python3

# Import necessary package
from os.path import expanduser
import rospkg

# Initialize home directory
home = expanduser("~")

def line_count(text_file):
	number_of_lines = 0
	file = open(text_file, "r")
	for line in file:
		line = line.strip("\n")
		if line == "":
			break
		else:
			number_of_lines = number_of_lines + 1
	file.close()
	return number_of_lines

# Read room from /catkin_ws/src/DELIVERY/coconut_deli_navigation/text/test.txt file and keep goals in list
# 1001: 3.11003708839, 1.53399014473, 0.716168884136, 0.697927022973 # roomname: px, py, oz, ow
def read_room():
	room_path = rospkg.RosPack().get_path("coconut_task_fsm")  ##"{}/catkin_ws/src/DELIVERY/coconut_deli_navigation/text/".format(home)
	room_filename = "task.txt"
	path = room_path + "/task/" + room_filename

	num_room = line_count(path)

	room_list = []
	split_roomList = []
	room_pose_list = []
	room_name_list = []

	fil_room_pose_list = []

	with open(path) as f:
		room_list = f.read().splitlines()
	
	## room name 
	for index in range(len(room_list)):
		if index < num_room:
			string = room_list[index]
			room_list[index] = string.split(":")
			split_roomList.append(room_list[index])
		if index >= num_room:
			pass
	for data in split_roomList:
		for index in range(len(data)):
			if index == 0:
				room_name_list.append(data[index])
			elif index == 1:
				room_pose_list.append(data[index])
	# print(room_name_list)

	## room pose
	for index in range(len(room_pose_list)):
	    if index < num_room:
	        string = room_pose_list[index]
	        room_pose_list[index] = string.split(",")
	        fil_room_pose_list.append(room_pose_list[index])
	    if index >= num_room:
	        pass

	for index in range(len(fil_room_pose_list)):
	    for index_sub in range(len(fil_room_pose_list[index])):
	        fil_room_pose_list[index][index_sub] = float(fil_room_pose_list[index][index_sub])
	# print(fil_room_pose_list)

	room_data = {}
	for index in range(0, len(fil_room_pose_list)):
		room_data.update({room_name_list[index]: fil_room_pose_list[index]})
	# print(room_data)

	return room_data

# room = read_room()
# print(room)
# print(room["1002"])
