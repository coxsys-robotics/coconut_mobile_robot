#!/usr/bin/env python

# Import necessary package
import rospy
import dynamic_reconfigure.client

# Open obstacle avoidance system
def open_obstacleAvoidance():
	client_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
	params_global = { 'enabled' : 'True' }
	client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
	params_local = { 'enabled' : 'True' }

	config_global = client_global.update_configuration(params_global)
	config_local = client_local.update_configuration(params_local)
	
# Close obstacle avoidance system
def close_obstacleAvoidance():
	client_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
	params_global = { 'enabled' : 'False' }
	client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
	params_local = { 'enabled' : 'False' }
	
	config_global = client_global.update_configuration(params_global)
	config_local = client_local.update_configuration(params_local)