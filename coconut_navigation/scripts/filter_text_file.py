#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from os.path import expanduser


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


def read_inflation_radius(filename):
	inflation_radius = 0
	file = open(filename)
	word = "inflation_radius"
	for line in file:
		line.strip().split('/n')
		if word in line:
			inflation_radius=line[line.find(":")+1:]
	inflation_radius = float(inflation_radius.strip(' '))
	inflation_radius = 0.1

	return inflation_radius


def get_image(filename, inflation_radius, resolution):
	img = cv2.imread(filename,0)
	# img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	kernel_params = int(((inflation_radius/resolution) * 2) + 1)
	print(kernel_params)
	kernel = np.ones((kernel_params, kernel_params), np.uint8) 
	img_erosion = cv2.erode(img.copy(), kernel, iterations=1) 
	# img_erosion = cv2.cvtColor(img_erosion, cv2.COLOR_BGR2RGB)
	
	return img_erosion, img


def read_goal(filename):
	num_goal = line_count(filename)
	fil_goalList = []
	with open(filename) as f:
		goal_list = f.read().splitlines()

	for index in range(len(goal_list)):
		if index < num_goal:
			string = goal_list[index]
			goal_list[index] = string.split(",")
			fil_goalList.append(goal_list[index])
		if index >= num_goal:
			pass

	return fil_goalList


def read_goal(filename):
	num_goal = line_count(filename)
	fil_goalList = []
	with open(filename) as f:
		goal_list = f.read().splitlines()

	for index in range(len(goal_list)):
		if index < num_goal:
			string = goal_list[index]
			goal_list[index] = string.split(",")
			fil_goalList.append(goal_list[index])
		if index >= num_goal:
			pass

	return fil_goalList


def read_yaml(filename):
	num_line = line_count(filename)
	split_yamlList = []

	with open(filename) as f:
		yaml_list = f.read().splitlines()
	yaml_list = [yaml_list[1], yaml_list[2]]

	for index in range(len(yaml_list)):
		string = yaml_list[index]
		yaml_list[index] = string.split(": ")
		split_yamlList.append(yaml_list[index])

	yaml_resolution = float(split_yamlList[0][1])
	
	yaml_origin = split_yamlList[1][1].strip('][').split(', ') 
	yaml_origin_xy = [float(yaml_origin[0]), float(yaml_origin[1])]

	return yaml_origin_xy, yaml_resolution


def remove_values_from_list(the_list, val):
   return [value for value in the_list if value != val]


def check_obstacles(origin_coordinate_meters, goal_coordinate_meters, resolution, image, image_erosion):
	list_goal_pixel = []
	able_to_move = []
	h, w = image.shape

	original_image = image_erosion.copy()
	filtered_image = image.copy()

	# ORIGIN
	x_origin = (origin_coordinate_meters[0]*-1)/resolution
	y_origin = h - ((origin_coordinate_meters[1]*-1)/resolution)
	x_origin = int(round(x_origin))
	y_origin = int(round(y_origin))
	# cv2.circle(original_image, (x_origin, y_origin), 5, (255,0,0), -1)
	# cv2.circle(filtered_image, (x_origin, y_origin), 5, (255,0,0), -1)

	# ORIGINAL GOAL
	for index in range(len(goal_coordinate_meters)):
		x = ((origin_coordinate_meters[0]*-1) + float(goal_coordinate_meters[index][1]))/resolution
		y = h - ((origin_coordinate_meters[1]*-1) + float(goal_coordinate_meters[index][2]))/resolution
		x = int(round(x))
		y = int(round(y))
		list_goal_pixel.append([x, y])
		# cv2.circle(original_image, (x, y), 5, (0,0,255), -1)

	# # OUTPUT ORIGINAL
	# cv2.namedWindow('Original Image', 0)
	# cv2.imshow('Original Image', original_image)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	for index in range(len(list_goal_pixel)):
		if image_erosion[list_goal_pixel[index][1], list_goal_pixel[index][0]] == 0:
			# print("BLACK")
			goal_coordinate_meters[index] = "OBSTACLE"
		elif image_erosion[list_goal_pixel[index][1], list_goal_pixel[index][0]] == 205:
			# print("GRAY")
			goal_coordinate_meters[index] = "OBSTACLE"
		elif image_erosion[list_goal_pixel[index][1], list_goal_pixel[index][0]] == 254:
			# print("WHITE")
			able_to_move.append(list_goal_pixel[index])
	
	# # ABLE TO MOVE GOAL
	# for index in range(len(able_to_move)):
	# 	cv2.circle(filtered_image, (able_to_move[index][0], able_to_move[index][1]), 5, (0,0,255), -1)

	if "OBSTACLE" in goal_coordinate_meters:
		goal_coordinate_meters = remove_values_from_list(goal_coordinate_meters, "OBSTACLE")
		goal_coordinate_meters_filtered = goal_coordinate_meters
	elif "OBSTACLE" not in goal_coordinate_meters:
		goal_coordinate_meters_filtered = goal_coordinate_meters

	# # OUTPUT ABLE TO MOVE
	# cv2.namedWindow('Filtered Image', 0)
	# cv2.imshow('Filtered Image', filtered_image)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	return goal_coordinate_meters_filtered


def write_goal(goal_coordinate_meters_filtered, filename):
	f = open(filename, "w")
	for index in range(len(goal_coordinate_meters_filtered)):
		f.write(goal_coordinate_meters_filtered[index][0] + "," + goal_coordinate_meters_filtered[index][1] + "," + 
				goal_coordinate_meters_filtered[index][2] + "," + goal_coordinate_meters_filtered[index][3] + "," + goal_coordinate_meters_filtered[index][4])
		f.write('\n')
	f.close()


def main():
	home = expanduser("~")

	# map_name = "samyan_mitrtown_20th_floor_nav"
	map_name = rospy.get_param("/map_name", "samyan_mitrtown_20th_floor_nav")

	yaml_path = "{}/coconut_ws/src/coconut_uvc_bringup/map/".format(home)
	yaml_filename = "{}.yaml".format(map_name)
	yaml_file = yaml_path + yaml_filename

	image_path = "{}/coconut_ws/src/coconut_uvc_bringup/map/".format(home)
	image_filename = "{}_nav.pgm".format(map_name)
	image_file = image_path + image_filename

	goal_path = "{}/coconut_ws/src/coconut_uvc_navigation/text/".format(home)
	goal_filename = "test.txt"
	goal_file = goal_path + goal_filename

	costmap_path = "{}/coconut_ws/src/coconut_uvc_bringup/teb_config/".format(home)
	costmap_filename = "local_costmap_params.yaml"
	costmap_file = costmap_path + costmap_filename

	inflation_radius = read_inflation_radius(costmap_file)
		
	origin_coordinate_meters, resolution = read_yaml(yaml_file)

	goal_coordinate_meters = read_goal(goal_file)
	
	image_erosion, image = get_image(image_file, inflation_radius, resolution)

	goal_coordinate_meters_filtered = check_obstacles(origin_coordinate_meters, goal_coordinate_meters, resolution, image, image_erosion)

	write_goal(goal_coordinate_meters_filtered, goal_file)


if __name__ == '__main__':
	main()
