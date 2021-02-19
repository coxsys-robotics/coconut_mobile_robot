#!/usr/bin/env python3

# Import necessarry package
import rospy
import dynamic_reconfigure.client

# Open obstacle avoidance system
def open_obstacleAvoidance():
	client_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
	params_global = { 'enabled' : 'True' }
	client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
	params_local = { 'enabled' : 'True' }
	
	recovery_behavior = dynamic_reconfigure.client.Client("/move_base")
	params_recovery = { 'recovery_behavior_enabled' : 'True' }

	clearing_rotation = dynamic_reconfigure.client.Client("/move_base")
	params_clearing = { 'clearing_rotation_allowed' : 'True' }

	config_global = client_global.update_configuration(params_global)
	config_local = client_local.update_configuration(params_local)
	config_recovery = recovery_behavior.update_configuration(params_recovery)
	config_clearing = clearing_rotation.update_configuration(params_clearing)

# Close obstacle avoidance system
def close_obstacleAvoidance():
	client_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
	params_global = { 'enabled' : 'False' }
	client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
	params_local = { 'enabled' : 'False' }
	
	recovery_behavior = dynamic_reconfigure.client.Client("/move_base")
	params_recovery = { 'recovery_behavior_enabled' : 'False' }

	clearing_rotation = dynamic_reconfigure.client.Client("/move_base")
	params_clearing = { 'clearing_rotation_allowed' : 'False' }

	config_global = client_global.update_configuration(params_global)
	config_local = client_local.update_configuration(params_local)
	config_recovery = recovery_behavior.update_configuration(params_recovery)
	config_clearing = clearing_rotation.update_configuration(params_clearing)