#!/usr/bin/env python

import rospy
from os.path import expanduser
import dynamic_reconfigure.client

costmap_common_interest_word_1 = "obstacle_range"
costmap_common_interest_word_2 = "raytrace_range"
costmap_common_interest_word_3 = "inflation_radius"
costmap_common_interest_word_4 = "cost_scaling_factor"

global_costmap_interest_word_1 = "inflation_radius"
global_costmap_interest_word_2 = "cost_scaling_factor"

local_costmap_interest_word_1 = "inflation_radius"
local_costmap_interest_word_2 = "cost_scaling_factor"

teb_local_planner_interest_word_1 = "max_vel_x_backwards"
teb_local_planner_interest_word_2 = "max_vel_x"
teb_local_planner_interest_word_3 = "max_vel_theta"
teb_local_planner_interest_word_4 = "acc_lim_x"
teb_local_planner_interest_word_5 = "acc_lim_theta"
teb_local_planner_interest_word_6 = "xy_goal_tolerance"
teb_local_planner_interest_word_7 = "yaw_goal_tolerance"


def read_costmap_common_yaml(filename):
	obstacle_range = 0
	raytrace_range = 0
	inflation_radius = 0
	cost_scaling_factor = 0
	
	costmap_common_line = 0
	obstacle_range_line = 0
	raytrace_range_line = 0
	inflation_radius_line = 0
	cost_scaling_factor_line = 0

	costmap_common_dic = {}

	file = open(filename)
	for line in file:
		costmap_common_line = costmap_common_line + 1
		line.strip().split('/n')
		if costmap_common_interest_word_1 in line:
			obstacle_range = line[line.find(":")+2:]
			obstacle_range = float(obstacle_range)
			obstacle_range_line = costmap_common_line
		if costmap_common_interest_word_2 in line:
			raytrace_range = line[line.find(":")+2:]
			raytrace_range = float(raytrace_range)
			raytrace_range_line = costmap_common_line
		if costmap_common_interest_word_3 in line:
			inflation_radius = line[line.find(":")+2:]
			inflation_radius = float(inflation_radius)
			inflation_radius_line = costmap_common_line
		if costmap_common_interest_word_4 in line:
			cost_scaling_factor = line[line.find(":")+2:]
			cost_scaling_factor = float(cost_scaling_factor)
			cost_scaling_factor_line = costmap_common_line
			break
	
	costmap_common_dic ={"obstacle_range":[obstacle_range, obstacle_range_line], "raytrace_range":[raytrace_range, raytrace_range_line], 
						"inflation_radius":[inflation_radius, inflation_radius_line], "cost_scaling_factor":[cost_scaling_factor, cost_scaling_factor_line]}
	
	print("CURRENT COSTMAP COMMON PARAMETERS")
	print("Obstacle range: %.2f at line %d" %(costmap_common_dic["obstacle_range"][0], costmap_common_dic["obstacle_range"][1]))
	print("Raytrace range: %.2f at line %d" %(costmap_common_dic["raytrace_range"][0], costmap_common_dic["raytrace_range"][1]))
	print("Inflation radius: %.2f at line %d " %(costmap_common_dic["inflation_radius"][0], costmap_common_dic["inflation_radius"][1]))
	print("Cost scaling factor: %.2f at line %d " %(costmap_common_dic["cost_scaling_factor"][0], costmap_common_dic["cost_scaling_factor"][1]))

	return costmap_common_dic

def dynamic_reconfigure_costmap_common():
	# OBSTACLE RANGE
	# client_costmap_common_obstacle_range_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
	client_costmap_common_obstacle_range_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
	params_costmap_common_obstacle_range = { 'obstacle_range' : user_costmap_common_obstacle_range }
	# config_obstacle_range_global = client_costmap_common_obstacle_range_global.update_configuration(params_costmap_common_obstacle_range)
	config_obstacle_range_local = client_costmap_common_obstacle_range_local.update_configuration(params_costmap_common_obstacle_range)
	
	# RAYTRACE RANGE
	# client_costmap_common_raytrace_range_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
	client_costmap_common_raytrace_range_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
	params_costmap_common_raytrace_range = { 'raytrace_range' : user_costmap_common_raytrace_range }
	# config_raytrace_range_global = client_costmap_common_raytrace_range_global.update_configuration(params_costmap_common_raytrace_range)
	config_raytrace_range_local = client_costmap_common_raytrace_range_local.update_configuration(params_costmap_common_raytrace_range)
	
	# INFLATION RADIUS
	# client_costmap_common_inflation_radius_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
	# client_costmap_common_inflation_radius_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
	# params_costmap_common_inflation_radius = { 'inflation_radius' : user_costmap_common_inflation_radius }
	# config_inflation_radius_global = client_costmap_common_inflation_radius_global.update_configuration(params_costmap_common_inflation_radius)
	# config_inflation_radius_local = client_costmap_common_inflation_radius_local.update_configuration(params_costmap_common_inflation_radius)

	# COST SCALING FACTOR
	# client_costmap_common_cost_scaling_factor_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
	# client_costmap_common_cost_scaling_factor_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
	# params_costmap_common_cost_scaling_factor = { 'cost_scaling_factor' : user_costmap_common_cost_scaling_factor }
	# config_cost_scaling_factor_global = client_costmap_common_cost_scaling_factor_global.update_configuration(params_costmap_common_cost_scaling_factor)
	# config_cost_scaling_factor_local = client_costmap_common_cost_scaling_factor_local.update_configuration(params_costmap_common_cost_scaling_factor)

def write_costmap_common_yaml(filename, costmap_common_dic):
	file = open(filename, "r")
	lines = file.readlines()
	lines[costmap_common_dic["obstacle_range"][1]-1] = "  obstacle_range: {}\n".format(user_costmap_common_obstacle_range)
	lines[costmap_common_dic["raytrace_range"][1]-1] = "  raytrace_range: {}\n".format(user_costmap_common_raytrace_range)
	lines[costmap_common_dic["inflation_radius"][1]-1] = "  inflation_radius: {}\n".format(user_costmap_common_inflation_radius)
	lines[costmap_common_dic["cost_scaling_factor"][1]-1] = "  cost_scaling_factor: {}\n".format(user_costmap_common_cost_scaling_factor)

	file = open(filename, "w")
	file.writelines(lines)
	file.close()


def read_global_costmap_yaml(filename):
	inflation_radius = 0
	cost_scaling_factor = 0
	
	global_costmap_line = 0
	inflation_radius_line = 0
	cost_scaling_factor_line = 0

	global_costmap_dic = {}

	file = open(filename)
	for line in file:
		global_costmap_line = global_costmap_line + 1
		line.strip().split('/n')
		if global_costmap_interest_word_1 in line:
			inflation_radius = line[line.find(":")+2:]
			inflation_radius = float(inflation_radius)
			inflation_radius_line = global_costmap_line
		if global_costmap_interest_word_2 in line:
			cost_scaling_factor = line[line.find(":")+2:]
			cost_scaling_factor = float(cost_scaling_factor)
			cost_scaling_factor_line = global_costmap_line
			break

	global_costmap_dic ={"inflation_radius":[inflation_radius, inflation_radius_line],
						 "cost_scaling_factor":[cost_scaling_factor, cost_scaling_factor_line]}
	
	print("CURRENT GLOBAL COSTMAP PARAMETERS")
	print("Inflation radius: %.2f at line %d " %(global_costmap_dic["inflation_radius"][0], global_costmap_dic["inflation_radius"][1]))
	print("Cost scaling factor: %.2f at line %d " %(global_costmap_dic["cost_scaling_factor"][0], global_costmap_dic["cost_scaling_factor"][1]))

	return global_costmap_dic

def dynamic_reconfigure_global_costmap():
	# INFLATION RADIUS
	client_global_costmap_inflation_radius_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer")
	params_global_costmap_inflation_radius = { 'inflation_radius' : user_global_costmap_inflation_radius }
	config_inflation_radius_global = client_global_costmap_inflation_radius_global.update_configuration(params_global_costmap_inflation_radius)

	# COST SCALING FACTOR
	client_global_costmap_cost_scaling_factor_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer")
	params_global_costmap_cost_scaling_factor = { 'cost_scaling_factor' : user_global_costmap_cost_scaling_factor }
	config_cost_scaling_factor_global = client_global_costmap_cost_scaling_factor_global.update_configuration(params_global_costmap_cost_scaling_factor)

def write_global_costmap_yaml(filename, global_costmap_dic):
	file = open(filename, "r")
	lines = file.readlines()
	lines[global_costmap_dic["inflation_radius"][1]-1] = "    inflation_radius: {}\n".format(user_global_costmap_inflation_radius)
	lines[global_costmap_dic["cost_scaling_factor"][1]-1] = "    cost_scaling_factor: {}\n".format(user_global_costmap_cost_scaling_factor)

	file = open(filename, "w")
	file.writelines(lines)
	file.close()


def read_local_costmap_yaml(filename):
	inflation_radius = 0
	cost_scaling_factor = 0
	
	local_costmap_line = 0
	inflation_radius_line = 0
	cost_scaling_factor_line = 0

	local_costmap_dic = {}

	file = open(filename)
	for line in file:
		local_costmap_line = local_costmap_line + 1
		line.strip().split('/n')
		if local_costmap_interest_word_1 in line:
			inflation_radius = line[line.find(":")+2:]
			inflation_radius = float(inflation_radius)
			inflation_radius_line = local_costmap_line
		if local_costmap_interest_word_2 in line:
			cost_scaling_factor = line[line.find(":")+2:]
			cost_scaling_factor = float(cost_scaling_factor)
			cost_scaling_factor_line = local_costmap_line
			break

	local_costmap_dic ={"inflation_radius":[inflation_radius, inflation_radius_line],
						 "cost_scaling_factor":[cost_scaling_factor, cost_scaling_factor_line]}
	
	print("CURRENT LOCAL COSTMAP PARAMETERS")
	print("Inflation radius: %.2f at line %d " %(local_costmap_dic["inflation_radius"][0], local_costmap_dic["inflation_radius"][1]))
	print("Cost scaling factor: %.2f at line %d " %(local_costmap_dic["cost_scaling_factor"][0], local_costmap_dic["cost_scaling_factor"][1]))

	return local_costmap_dic

def dynamic_reconfigure_local_costmap():
	# INFLATION RADIUS
	client_local_costmap_inflation_radius_global = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer")
	params_local_costmap_inflation_radius = { 'inflation_radius' : user_local_costmap_inflation_radius }
	config_inflation_radius_global = client_local_costmap_inflation_radius_global.update_configuration(params_local_costmap_inflation_radius)

	# COST SCALING FACTOR
	client_local_costmap_cost_scaling_factor_global = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer")
	params_local_costmap_cost_scaling_factor = { 'cost_scaling_factor' : user_local_costmap_cost_scaling_factor }
	config_cost_scaling_factor_global = client_local_costmap_cost_scaling_factor_global.update_configuration(params_local_costmap_cost_scaling_factor)

def write_local_costmap_yaml(filename, local_costmap_dic):
	file = open(filename, "r")
	lines = file.readlines()
	lines[local_costmap_dic["inflation_radius"][1]-1] = "    inflation_radius: {}\n".format(user_local_costmap_inflation_radius)
	lines[local_costmap_dic["cost_scaling_factor"][1]-1] = "    cost_scaling_factor: {}\n".format(user_local_costmap_cost_scaling_factor)

	file = open(filename, "w")
	file.writelines(lines)
	file.close()


def read_teb_local_planner_yaml(filename):
	max_vel_x = 0
	max_vel_x_backwards = 0
	max_vel_theta = 0
	acc_lim_x = 0
	acc_lim_theta = 0
	xy_goal_tolerance = 0
	yaw_goal_tolerance = 0
	
	teb_local_planner_line = 0
	max_vel_x_line = 0
	max_vel_x_backwards_line = 0
	max_vel_theta_line = 0
	acc_lim_x_line = 0
	acc_lim_theta_line = 0
	xy_goal_tolerance_line = 0
	yaw_goal_tolerance_line = 0

	teb_local_planner_dic = {}

	file = open(filename)
	for line in file:
		teb_local_planner_line = teb_local_planner_line + 1
		line.strip().split('/n')
		if teb_local_planner_interest_word_1 in line:
			max_vel_x_backwards = line[line.find(":")+2:]
			max_vel_x_backwards = float(max_vel_x_backwards)
			max_vel_x_backwards_line = teb_local_planner_line
		if teb_local_planner_interest_word_2 in line:
			max_vel_x = line[line.find(":")+2:]
			max_vel_x = float(max_vel_x)
			max_vel_x_line = teb_local_planner_line
		if teb_local_planner_interest_word_3 in line:
			max_vel_theta = line[line.find(":")+2:]
			max_vel_theta = float(max_vel_theta)
			max_vel_theta_line = teb_local_planner_line
		if teb_local_planner_interest_word_4 in line:
			acc_lim_x = line[line.find(":")+2:]
			acc_lim_x = float(acc_lim_x)
			acc_lim_x_line = teb_local_planner_line
		if teb_local_planner_interest_word_5 in line:
			acc_lim_theta = line[line.find(":")+2:]
			acc_lim_theta = float(acc_lim_theta)
			acc_lim_theta_line = teb_local_planner_line
		if teb_local_planner_interest_word_6 in line:
			xy_goal_tolerance = line[line.find(":")+2:]
			xy_goal_tolerance = float(xy_goal_tolerance)
			xy_goal_tolerance_line = teb_local_planner_line
		if teb_local_planner_interest_word_7 in line:
			yaw_goal_tolerance = line[line.find(":")+2:]
			yaw_goal_tolerance = float(yaw_goal_tolerance)
			yaw_goal_tolerance_line = teb_local_planner_line
			break
	
	teb_local_planner_dic ={"max_vel_x":[max_vel_x, max_vel_x_line],
							"max_vel_x_backwards":[max_vel_x_backwards, max_vel_x_backwards_line], 
							"max_vel_theta":[max_vel_theta, max_vel_theta_line], 
							"acc_lim_x":[acc_lim_x, acc_lim_x_line],
							"acc_lim_theta":[acc_lim_theta, acc_lim_theta_line], 
							"xy_goal_tolerance":[xy_goal_tolerance, xy_goal_tolerance_line], 
							"yaw_goal_tolerance":[yaw_goal_tolerance, yaw_goal_tolerance_line]}
	
	print("CURRENT TEB LOCAL PLANNER PARAMETERS")
	print("Maximum velocity backward: %.2f at line %d" %(teb_local_planner_dic["max_vel_x_backwards"][0], teb_local_planner_dic["max_vel_x_backwards"][1]))
	print("Maximum velocity x: %.2f at line %d" %(teb_local_planner_dic["max_vel_x"][0], teb_local_planner_dic["max_vel_x"][1]))
	print("Maximum velocity theta: %.2f at line %d " %(teb_local_planner_dic["max_vel_theta"][0], teb_local_planner_dic["max_vel_theta"][1]))
	print("Accerelation limit x: %.2f at line %d " %(teb_local_planner_dic["acc_lim_x"][0], teb_local_planner_dic["acc_lim_x"][1]))
	print("Accerelation limit theta: %.2f at line %d" %(teb_local_planner_dic["acc_lim_theta"][0], teb_local_planner_dic["acc_lim_theta"][1]))
	print("XY goal tolerance: %.2f at line %d " %(teb_local_planner_dic["xy_goal_tolerance"][0], teb_local_planner_dic["xy_goal_tolerance"][1]))
	print("Yaw goal tolerance: %.2f at line %d " %(teb_local_planner_dic["yaw_goal_tolerance"][0], teb_local_planner_dic["yaw_goal_tolerance"][1]))

	return teb_local_planner_dic

def dynamic_reconfigure_teb_local_planner():
	# MAXIMUM VELOCITY X
	client_teb_local_planner_max_vel_x = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
	params_teb_local_planner_max_vel_x = { 'max_vel_x' : user_teb_local_planner_max_vel_x }
	config_max_vel_x_local = client_teb_local_planner_max_vel_x.update_configuration(params_teb_local_planner_max_vel_x)
	
	# MAXIMUM VELOCITY X BACKWARD
	client_teb_local_planner_max_vel_x_backwards = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
	params_teb_local_planner_max_vel_x_backwards = { 'max_vel_x_backwards' : user_teb_local_planner_max_vel_x_backwards }
	config_max_vel_x_backwards_local = client_teb_local_planner_max_vel_x_backwards.update_configuration(params_teb_local_planner_max_vel_x_backwards)

	# MAXIMUM VELOCITY THETA
	client_teb_local_planner_max_vel_theta = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
	params_teb_local_planner_max_vel_theta = { 'max_vel_theta' : user_teb_local_planner_max_vel_theta }
	config_max_vel_theta_local = client_teb_local_planner_max_vel_theta.update_configuration(params_teb_local_planner_max_vel_theta)
	
	# ACCELERATION LIMIT X
	client_teb_local_planner_acc_lim_x = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
	params_teb_local_planner_acc_lim_x = { 'acc_lim_x' : user_teb_local_planner_acc_lim_x }
	config_acc_lim_x_local = client_teb_local_planner_acc_lim_x.update_configuration(params_teb_local_planner_acc_lim_x)

	# ACCLERATION LIMIT THETA
	client_teb_local_planner_acc_lim_theta = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
	params_teb_local_planner_acc_lim_theta = { 'acc_lim_theta' : user_teb_local_planner_acc_lim_theta }
	config_acc_lim_theta_local = client_teb_local_planner_acc_lim_theta.update_configuration(params_teb_local_planner_acc_lim_theta)
	
	# XY GOAL TOLERANCE
	client_teb_local_planner_xy_goal_tolerance = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
	params_teb_local_planner_xy_goal_tolerance = { 'xy_goal_tolerance' : user_teb_local_planner_xy_goal_tolerance }
	config_xy_goal_tolerance_local = client_teb_local_planner_xy_goal_tolerance.update_configuration(params_teb_local_planner_xy_goal_tolerance)

	# YAW GOAL TOLERANCE
	client_teb_local_planner_yaw_goal_tolerance = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS")
	params_teb_local_planner_yaw_goal_tolerance = { 'yaw_goal_tolerance' : user_teb_local_planner_yaw_goal_tolerance }
	config_yaw_goal_tolerance_local = client_teb_local_planner_yaw_goal_tolerance.update_configuration(params_teb_local_planner_yaw_goal_tolerance)

def write_teb_local_planner_yaml(filename, teb_local_planner_dic):
	file = open(filename, "r")
	lines = file.readlines()
	lines[teb_local_planner_dic["max_vel_x"][1]-1] = " max_vel_x: {}\n".format(user_teb_local_planner_max_vel_x)
	lines[teb_local_planner_dic["max_vel_x_backwards"][1]-1] = " max_vel_x_backwards: {}\n".format(user_teb_local_planner_max_vel_x_backwards)
	lines[teb_local_planner_dic["max_vel_theta"][1]-1] = " max_vel_theta: {}\n".format(user_teb_local_planner_max_vel_theta)
	lines[teb_local_planner_dic["acc_lim_x"][1]-1] = " acc_lim_x: {}\n".format(user_teb_local_planner_acc_lim_x)
	lines[teb_local_planner_dic["acc_lim_theta"][1]-1] = " acc_lim_theta: {}\n".format(user_teb_local_planner_acc_lim_theta)
	lines[teb_local_planner_dic["xy_goal_tolerance"][1]-1] = " xy_goal_tolerance: {}\n".format(user_teb_local_planner_xy_goal_tolerance)
	lines[teb_local_planner_dic["yaw_goal_tolerance"][1]-1] = " yaw_goal_tolerance: {}\n".format(user_teb_local_planner_yaw_goal_tolerance)

	file = open(filename, "w")
	file.writelines(lines)
	file.close()


def main():
	home = expanduser("~")

	##### Costmap common parameters #####
	yaml_costmap_common_name = "costmap_common"
	user_costmap_common_obstacle_range = 5.5 #5.5
	user_costmap_common_raytrace_range = 6.0 #6.0
	user_costmap_common_inflation_radius = 0.32 #0.32
	user_costmap_common_cost_scaling_factor = 1.0 #1.0

	##### Global costmap parameters #####
	yaml_global_costmap_name = "global_costmap"
	user_global_costmap_inflation_radius = 0.7 #0.7
	user_global_costmap_cost_scaling_factor = 0.5 #0.5

	##### Local costmap parameters #####
	yaml_local_costmap_name = "local_costmap"
	user_local_costmap_inflation_radius = 0.7 #0.7
	user_local_costmap_cost_scaling_factor = 0.5 #0.5
	
	##### Teb local planner parameters #####
	yaml_teb_local_planner_name = "teb_local_planner"
	
	user_teb_local_planner_max_vel_x_backwards = 0.2 #0.2
	user_teb_local_planner_max_vel_x = 0.2 #0.2
	user_teb_local_planner_max_vel_theta = 0.3 #0.3
	user_teb_local_planner_acc_lim_x = 0.2 #0.2
	user_teb_local_planner_acc_lim_theta = 0.2 #0.2
	user_teb_local_planner_xy_goal_tolerance = 0.05 #0.05
	user_teb_local_planner_yaw_goal_tolerance = 0.05 #0.05

	################################### COSTMAP COMMON ################################### 
	costmap_common_path = "{}/coconut_ws/src/UVC/coconut_navigation/teb_config/".format(home)
	costmap_common_filename = "{}_params.yaml".format(yaml_costmap_common_name)
	costmap_common_file = costmap_common_path + costmap_common_filename

	costmap_common_dic = read_costmap_common_yaml(costmap_common_file)
	dynamic_reconfigure_costmap_common()
	write_costmap_common_yaml(costmap_common_file, costmap_common_dic)

	print("\nUSER COSTMAP COMMON PARAMETERS")
	print("Obstacle range: %.2f at line %d" %(user_costmap_common_obstacle_range, costmap_common_dic["obstacle_range"][1]))
	print("Raytrace range: %.2f at line %d" %(user_costmap_common_raytrace_range, costmap_common_dic["raytrace_range"][1]))
	print("Inflation radius: %.2f at line %d " %(user_costmap_common_inflation_radius, costmap_common_dic["inflation_radius"][1]))
	print("Cost scaling factor: %.2f at line %d\n" %(user_costmap_common_cost_scaling_factor, costmap_common_dic["cost_scaling_factor"][1]))


	################################### GLOBAL COSTMAP ###################################
	global_costmap_path = "{}/coconut_ws/src/UVC/coconut_navigation/teb_config/".format(home)
	global_costmap_filename = "{}_params.yaml".format(yaml_global_costmap_name)
	global_costmap_file = global_costmap_path + global_costmap_filename

	global_costmap_dic = read_global_costmap_yaml(global_costmap_file)
	dynamic_reconfigure_global_costmap()
	write_global_costmap_yaml(global_costmap_file, global_costmap_dic)

	print("\nUSER GLOBAL COSTMAP PARAMETERS")
	print("Inflation radius: %.2f at line %d " %(user_global_costmap_inflation_radius, global_costmap_dic["inflation_radius"][1]))
	print("Cost scaling factor: %.2f at line %d\n" %(user_global_costmap_cost_scaling_factor, global_costmap_dic["cost_scaling_factor"][1]))


	################################### LOCAL COSTMAP ###################################
	local_costmap_path = "{}/coconut_ws/src/UVC/coconut_navigation/teb_config/".format(home)
	local_costmap_filename = "{}_params.yaml".format(yaml_local_costmap_name)
	local_costmap_file = local_costmap_path + local_costmap_filename

	local_costmap_dic = read_local_costmap_yaml(local_costmap_file)
	# dynamic_reconfigure_local_costmap(local_costmap_dic)
	write_local_costmap_yaml(local_costmap_file, local_costmap_dic)

	print("\nUSER LOCAL COSTMAP PARAMETERS")
	print("Inflation radius: %.2f at line %d " %(user_local_costmap_inflation_radius, local_costmap_dic["inflation_radius"][1]))
	print("Cost scaling factor: %.2f at line %d" %(user_local_costmap_cost_scaling_factor, local_costmap_dic["cost_scaling_factor"][1]))


	################################### TEB LOCAL PLANNER ###################################
	teb_local_planner_path = "{}/coconut_ws/src/UVC/coconut_navigation/teb_config/".format(home)
	teb_local_planner_filename = "{}_params.yaml".format(yaml_teb_local_planner_name)
	teb_local_planner_file = teb_local_planner_path + teb_local_planner_filename

	teb_local_planner_dic = read_teb_local_planner_yaml(teb_local_planner_file)
	# dynamic_reconfigure_teb_local_planner(teb_local_planner_dic)
	write_teb_local_planner_yaml(teb_local_planner_file, teb_local_planner_dic)

	print("\nUSER TEB LOCAL PLANNER PARAMETERS")
	print("Maximum velocity backward: %.2f at line %d" %(user_teb_local_planner_max_vel_x_backwards, teb_local_planner_dic["max_vel_x_backwards"][1]))
	print("Maximum velocity x: %.2f at line %d" %(user_teb_local_planner_max_vel_x, teb_local_planner_dic["max_vel_x"][1]))
	print("Maximum velocity theta: %.2f at line %d " %(user_teb_local_planner_max_vel_theta, teb_local_planner_dic["max_vel_theta"][1]))
	print("Accerelation limit x: %.2f at line %d" %(user_teb_local_planner_acc_lim_x, teb_local_planner_dic["acc_lim_x"][1]))
	print("Accerelation limit theta: %.2f at line %d" %(user_teb_local_planner_acc_lim_theta, teb_local_planner_dic["acc_lim_theta"][1]))
	print("XY goal tolerance: %.2f at line %d " %(user_teb_local_planner_xy_goal_tolerance, teb_local_planner_dic["xy_goal_tolerance"][1]))
	print("Yaw goal tolerance: %.2f at line %d" %(user_teb_local_planner_yaw_goal_tolerance, teb_local_planner_dic["yaw_goal_tolerance"][1]))


if __name__ == '__main__':
	main()