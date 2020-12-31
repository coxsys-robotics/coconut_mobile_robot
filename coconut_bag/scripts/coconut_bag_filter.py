#!/usr/bin/env python 

"""
filter ros bag file to use for offline slam
"""

import rosbag
rosbag.rosbag_main.filter_cmd(["coxsys.bag","coxsys_filtered.bag",
                               "topic != '/tf' or topic == '/tf' and m.transforms[0].header.frame_id != 'map' and m.transforms[0].child_frame_id != 't265_odom_frame'"])