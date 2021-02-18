#!/usr/bin/env python3

# Import necessary package
import sys
import rospy
from coconut_service.srv import *

# Function for clear cost map for navigation system
def clear_costmap_client():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_cost_map = rospy.ServiceProxy('/move_base/clear_costmaps', ClearCostMap)
        resp1 = clear_cost_map()
        return resp1
    except( rospy.ServiceException, e):
        print("Service call failed: %s"%e)
    finally:
        rospy.signal_shutdown("Exit")


if __name__ == "__main__":
    # Define node name
    rospy.init_node('clear_cost_map_node', disable_signals=True)
    clear_costmap_client()
