#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from shutil import copyfile
from os.path import expanduser

# format:
#     [ { "type": <0-black_li, 1-white_li, 2-black_po, 3-white_po>, "coordinate": [ [x_1, y_1], [x_2, y_2], [x_3, y_3], .. ]  }, 
#       { }, 
#       { }, 
#       ... 
#     ]        

def get_coordianate():
	edit_data = [{ "type": 0, "coordinate": [ [520, 520], [600, 600] ] }, 
	  			 { "type": 1, "coordinate": [ [30, 30], [40, 40], [30, 50], [40, 60] ] }, 
	  			 { "type": 2, "coordinate": [ [50, 50], [50, 150], [150, 150], [150, 50] ] }, 
	  			 { "type": 3, "coordinate": [ [410, 405], [420, 430], [470, 420], [450, 410] ] } 
				]     

	return edit_data

def get_image(filename):
	image = cv2.imread(filename, 0)	
	
	return image

def draw_polygon(image, points, color):
	points = np.array(points)
	image = cv2.fillPoly(image, [points], color)
	
	return image

def draw_line(image, points, color, thickness):
	for index in range(len(points)):
		points[index] = tuple(points[index])

	for index in range(1, len(points)):	
		image = cv2.line(image, points[index-1], points[index], color, thickness)
	
	return image

def save_image(filename, nav_image):
	cv2.imwrite(filename, nav_image) 

def create_yaml(yaml_file, new_yaml_file):
	copyfile(yaml_file, new_yaml_file)

def write_yaml(new_yaml_file, yaml_path, map_name):
	file = open(new_yaml_file, "r")
	lines = file.readlines()
	lines[0] = "image: {}{}_nav.pgm\n".format(yaml_path, map_name)
	file = open(new_yaml_file, "w")
	file.writelines(lines)
	file.close()

def edit_function_selection(edit_data ,image):
	for data in edit_data:
		if data["type"] == 0:
			image = draw_line(image, data["coordinate"], (0, 0, 0), 4)
		elif data["type"] == 1:
			image = draw_line(image, data["coordinate"], (254, 254, 254), 4)
		elif data["type"] == 2:
			image = draw_polygon(image, data["coordinate"], (0, 0, 0))
		elif data["type"] == 3:
			image = draw_polygon(image, data["coordinate"], (254, 254, 254))

	return image

def main(edit_data):
	home = expanduser("~")

	# map_name = "fortuna_4th_floor_nav"
	map_name = rospy.get_param("/map_name", "fortuna_4th_floor")

	image_path = "{}/coconut_ws/src/coconut_bringup/map/".format(home)
	image_filename = "{}.pgm".format(map_name)
	image_file = image_path + image_filename

	if "_nav" in map_name:
		nav_image_filename = "{}.pgm".format(map_name)
	elif "_nav" not in map_name:
		nav_image_filename = "{}_nav.pgm".format(map_name)

	nav_image_file = image_path + nav_image_filename

	image = get_image(image_file)

	# cv2.namedWindow('Image', 0)
	# cv2.imshow('Image', image)
	# cv2.waitKey(0)

	# cv2.destroyAllWindows()

	# edit_data = get_coordianate()

	nav_image = edit_function_selection(edit_data, image)

	# cv2.namedWindow('Nav Image', 0)
	# cv2.imshow('Nav Image', nav_image)
	# cv2.waitKey(0)

	# cv2.destroyAllWindows()

	save_image(nav_image_file, nav_image)

	yaml_path = "{}/coconut_ws/src/coconut_bringup/map/".format(home)
	yaml_filename = "{}.yaml".format(map_name)
	yaml_file = yaml_path + yaml_filename

	new_yaml_filename = "{}_nav.yaml".format(map_name)
	new_yaml_file = yaml_path + new_yaml_filename

	create_yaml(yaml_file, new_yaml_file)
	write_yaml(new_yaml_file ,yaml_path, map_name)


if __name__ == '__main__':
	main()