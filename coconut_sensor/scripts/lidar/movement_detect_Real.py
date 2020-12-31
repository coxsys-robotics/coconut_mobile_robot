# import rosbag
import math
import matplotlib.pyplot as plt
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose


"""
detect movement when robot is stationary.
"""

# num = 0

# data_points_x = []
# data_points_y = []
# data_points_ranges = []

# ptx = []
# pty = []

# new_list = []
# old_list = []
# idx_point_list = []

# mirror_rad_list = []
# mirror_range_list = []
# idx_mirror_list = []
# idx_mirror_list_new = []
# inds = []

# mirror_point = [(0,0),(0,10),(0,15)]
# origin = [-1.75/0.05,-1.6/0.05]

# def mirror_find(x,y): #rviz +x = -x in cv
#     # Find Ref mirror aranges
#     for mirror2_point in mirror_point:
#         range_mirror = math.sqrt(pow(-x-mirror2_point[0],2) + pow(-y-mirror2_point[1],2))
#         theta_mirror = math.atan2(mirror2_point[1], mirror2_point[0])
#         radian_mirror = theta_mirror * (math.pi/180)
                
#         mirror_range_list.append(range_mirror)
#         mirror_rad_list.append(radian_mirror)
#         idx_mirror_list.append(round(theta_mirror/0.2143))
                
#         # print("range_mirror = %.3f, idx_theta_mirror = %.3f, radian_mirror = %.3f" %(range_mirror, round(theta_mirror/0.2143), radian_mirror))

# def append_zero_point(ranges):
#     if idx_point_list != []:
#         for idx in idx_point_list:
#             if ranges[idx] != (0,0):
#                 old_list[idx] = ranges[idx]

# def callback(data):
#     ranges = data.ranges
#     pose = data.pose_from_robot
    

# bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-02-20-27-45.bag') #walking in
# # bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-02-20-29-44.bag') #walking in
# # bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-01-21-06-12.bag') #walking in
# # bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-01-21-05-36.bag') #door open & walking in
# # bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-02-20-33-19.bag') #door open & walking in(slow detect because stick around the door)
# # bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-01-20-59-28.bag') #walking???
# # bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-01-21-04-27.bag') #WTF why robot spinning
# # bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-01-21-03-20.bag') #walking in
# # bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-09/2020-07-09-20-47-03.bag') #walking in(human don't know)
# # bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-09/2020-07-09-20-45-59.bag') #walking out
# # bag = rosbag.Bag('/home/sun/catkin_ws/src/safty_detect/rosbag/2020-07-09/2020-07-09-20-44-55.bag') #walking out

# #this function should be origin from msg.pose, But this is for debug in the future should be(msg.pose[0], msg.pose[1])
# mirror_find(-origin[0],-origin[1])

# # mirror_range_list = set(mirror_range_list)
# # mirror_range_list = list(mirror_range_list)
# # idx_mirror_list = set(idx_mirror_list)
# # idx_mirror_list = list(idx_mirror_list)

# # print(mirror_range_list, idx_mirror_list)

# for i, mirror_idx in enumerate(idx_mirror_list):
#     if mirror_idx not in idx_mirror_list_new:
#         inds.append(i)    
#         idx_mirror_list_new.append(mirror_idx)
        
# # print("idx_mirror_list = ", idx_mirror_list)
# # print("idx_mirror_list_new = ", idx_mirror_list_new)
# # print("inds = ", inds)

# new_mirror_range = []
# new_mirror_order = []

# for i in inds:
#     new_mirror_range.append(mirror_range_list[i])
#     new_mirror_order.append(idx_mirror_list[i])
# # print(new_mirror_range, new_mirror_order)


# for topic, msg, t in bag.read_messages(topics=['/scan','/pose_from_robot']):
# # if __name__ == "__main__":
# #     rospy.init_node("lida_sub")
# #     rospy.Subscriber("/scan", LaserScan, callback)
# #     rospy.Subscriber("/pose_from_robot", poseFromRobot, callback)
# #     rospy.spin()
    
#     if topic == "/scan" and msg.ranges:
#         num += 1
#         radians = 2*math.pi/float(len(msg.ranges))
#         resolution = 0.05   #This resolution from yaml ---> change here for yaml code <---
#         for idx in range(len(msg.ranges)):
#             x = (msg.ranges[idx]/resolution)*math.cos(radians*idx)
#             y = (msg.ranges[idx]/resolution)*math.sin(radians*idx)
#             ranges = (msg.ranges[idx]/resolution)
#             data_points_x.append(x)
#             data_points_y.append(y)
#             data_points_ranges.append(ranges)
#             data = (x,y,ranges)
#             new_list.append(data)
#             # print("x = %.3f, y = %.3f" %(x,y))
        
#         if old_list == []:  #Get datas from new_list 1 turns.
#             old_list = new_list
        
#         if num <= 10: #construct map for 5 msg
#             for idx_point in range(len(data_points_x)-1):
#                 if abs(old_list[idx_point][0]) == 0.0 and abs(old_list[idx_point][1]) == 0.0: #Check (0,0) in new_list
#                     idx_point_list.append(idx_point)
#             append_zero_point(new_list)
            
#         new_list = []
#         ptx = data_points_x
#         pty = data_points_y
#         data_points_x = []  #reset x, y list
#         data_points_y = []  
#         data_points_ranges = []
        
#         # print(msg.ranges[1]/resolution)
       
#         if num > 11:
#             for idx_old_point in range(len(old_list)-1):
#                 if old_list[idx_old_point][2]-(msg.ranges[idx_old_point]/resolution) >= 15.0 and msg.ranges[idx_old_point]/resolution != 0.0:
#                     for order in inds:
#                         # print("msg.range", msg.ranges[order]/resolution)
#                         # print("inds ", new_mirror_range[order])
#                         if msg.ranges[order]/resolution >= new_mirror_range[order]:
#                             print("Condition Mirror")
#                         elif msg.ranges[order]/resolution < new_mirror_range[order]:
#                             print("Condition Walk-In")
#                             print("num : %d" %(num))
#                             print("idx : %d" %(idx_old_point))
#                             print("old : %.3f" %(old_list[idx_old_point][2]))
#                             print("new : %.3f" %(msg.ranges[idx_old_point]/resolution))
#                             print('\033[91m' + "warnning!" + '\033[0m')
#                 elif (msg.ranges[idx_old_point]/resolution)-old_list[idx_old_point][2] >= 25.0 and old_list[idx_old_point][2] != 0.0:
#                     for order in inds:
#                         if msg.ranges[order]/resolution >= new_mirror_range[order]:
#                             print("Condition Mirror")
#                         elif msg.ranges[order]/resolution < new_mirror_range[order]:
#                             print("Condition Walk-In")
#                             print("num : %d" %(num))
#                             print("idx : %d" %(idx_old_point))
#                             print("old : %.3f" %(old_list[idx_old_point][2]))
#                             print("new : %.3f" %(msg.ranges[idx_old_point]/resolution))
#                             print('\033[91m' + "warnning!" + '\033[0m')
#             if num % 50 == 0:
#                 plt.clf()       
#             plt.scatter(0,0) 
#             plt.scatter(0,10) 
#             plt.scatter(0,15) 
#             plt.scatter(ptx,pty)
#             plt.pause(0.25)                  

# plt.show()
# bag.close()


class movement_detector():
    def __init__(self):
        self.num = 0

        self.data_points_x = []
        self.data_points_y = []
        self.data_points_ranges = []

        self.ptx = []
        self.pty = []

        self.new_list = []
        self.old_list = []
        self.idx_point_list = []

        self.mirror_rad_list = []
        self.mirror_range_list = []
        self.idx_mirror_list = []
        self.idx_mirror_list_new = []
        self.inds = []

        self.new_mirror_range = []
        self.new_mirror_order = []

        self.mirror_point = [(0,0),(0,10),(0,15)]
        self.pose_store = []
        self.num_average_pose = 5
        self.origin = [0, 0]

        self.lidar_scan = LaserScan()

        self.get_lidar_msg = False
        self.get_pose = False

        rospy.init_node("lida_sub")
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.pose_sub = rospy.Subscriber("/pose_from_robot", Pose, self.pose_callback)

    def lidar_callback(self, data):
        self.lidar_scan = data
        self.get_lidar_msg = True

    def pose_callback(self, data):
        # Get robot's current position & orientation. 
        # Average from n msg for more accuracy.
        self.pose_store.append( (data.position.x, data.position.y) )
        if(len(self.pose_store) == self.num_average_pose):
            self.pose_sub.unregister()
            x = y = 0
            for position in self.pose_store:
                x += position[0]
                y += position[1]
            self.origin = [ x/self.num_average_pose, y/self.num_average_pose ]    
            self.mirror_find(self.origin[0], self.origin[1])
            self.get_pose = True

    def mirror_find(self, x, y): #rviz +x = -x in cv
        # Find distance from robot to mirror.
        # x, y correspond to robot's (x,y) position
        for mirror2_point in self.mirror_point:
            range_mirror = math.sqrt(pow(-x-mirror2_point[0],2) + pow(-y-mirror2_point[1],2))
            theta_mirror = math.atan2(mirror2_point[1], mirror2_point[0])
            radian_mirror = theta_mirror * (math.pi/180)
                    
            self.mirror_range_list.append(range_mirror)
            self.mirror_rad_list.append(radian_mirror)
            self.idx_mirror_list.append(round(theta_mirror/0.2143))
                    
            # print("range_mirror = %.3f, idx_theta_mirror = %.3f, radian_mirror = %.3f" %(range_mirror, round(theta_mirror/0.2143), radian_mirror))

    def append_zero_point(self,ranges):
        if self.idx_point_list != []:
            for idx in self.idx_point_list:
                if ranges[idx] != (0,0):
                    self.old_list[idx] = ranges[idx]

    def main(self):
        for i, mirror_idx in enumerate(self.idx_mirror_list):
            if mirror_idx not in self.idx_mirror_list_new:
                self.inds.append(i)    
                self.idx_mirror_list_new.append(mirror_idx)
                
        # print("idx_mirror_list = ", idx_mirror_list)
        # print("idx_mirror_list_new = ", idx_mirror_list_new)
        # print("inds = ", inds)

        for i in self.inds:
            self.new_mirror_range.append(self.mirror_range_list[i])
            self.new_mirror_order.append(self.idx_mirror_list[i])
        # print(new_mirror_range, new_mirror_order)

        if self.get_lidar_msg and self.get_pose:
            self.num += 1
            radians = 2*math.pi/float(len(self.lidar_scan.ranges))
            resolution = 0.05   #This resolution from yaml ---> change here for yaml code <---
            for idx in range(len(self.lidar_scan.ranges)):
                x = (self.lidar_scan.ranges[idx]/resolution)*math.cos(radians*idx)
                y = (self.lidar_scan.ranges[idx]/resolution)*math.sin(radians*idx)
                ranges = (self.lidar_scan.ranges[idx]/resolution)
                self.data_points_x.append(x)
                self.data_points_y.append(y)
                self.data_points_ranges.append(ranges)
                data = (x,y,ranges)
                self.new_list.append(data)
                # print("x = %.3f, y = %.3f" %(x,y))
            
            if self.old_list == []:  #Get datas from new_list 1 turns.
                self.old_list = self.new_list
            
            if self.num <= 10: #construct map for 5 msg
                for idx_point in range(len(self.data_points_x)-1):
                    if abs(self.old_list[idx_point][0]) == 0.0 and abs(self.old_list[idx_point][1]) == 0.0: #Check (0,0) in new_list
                        self.idx_point_list.append(idx_point)
                self.append_zero_point(self.new_list)
                
            self.new_list = []
            self.ptx = self.data_points_x
            self.pty = self.data_points_y
            self.data_points_x = []  #reset x, y list
            self.data_points_y = []  
            self.data_points_ranges = []
            
            # print(msg.ranges[1]/resolution)
        
            if self.num > 11:
                for self.idx_old_point in range(len(self.old_list)-1):
                    if self.old_list[self.idx_old_point][2]-(self.lidar_scan.ranges[self.idx_old_point]/resolution) >= 15.0 and self.lidar_scan.ranges[self.idx_old_point]/resolution != 0.0:
                        for order in self.inds:
                            # print("msg.range", msg.ranges[order]/resolution)
                            # print("inds ", new_mirror_range[order])
                            if self.lidar_scan.ranges[order]/resolution >= self.new_mirror_range[order]:
                                print("Condition Mirror")
                            elif self.lidar_scan.ranges[order]/resolution < self.new_mirror_range[order]:
                                print("Condition Walk-In")
                                print("num : %d" %(self.num))
                                print("idx : %d" %(self.idx_old_point))
                                print("old : %.3f" %(self.old_list[self.idx_old_point][2]))
                                print("new : %.3f" %(self.lidar_scan.ranges[self.idx_old_point]/resolution))
                                print('\033[91m' + "warnning!" + '\033[0m')
                    elif (self.lidar_scan.ranges[self.idx_old_point]/resolution)-self.old_list[self.idx_old_point][2] >= 25.0 and self.old_list[self.idx_old_point][2] != 0.0:
                        for order in self.inds:
                            if self.lidar_scan.ranges[order]/resolution >= self.new_mirror_range[order]:
                                print("Condition Mirror")
                            elif self.lidar_scan.ranges[order]/resolution < self.new_mirror_range[order]:
                                print("Condition Walk-In")
                                print("num : %d" %(self.num))
                                print("idx : %d" %(self.idx_old_point))
                                print("old : %.3f" %(self.old_list[self.idx_old_point][2]))
                                print("new : %.3f" %(self.lidar_scan.ranges[self.idx_old_point]/resolution))
                                print('\033[91m' + "warnning!" + '\033[0m')
                if self.num % 50 == 0:
                    plt.clf()      
                     
                plt.scatter(0,0) 
                plt.scatter(0,10) 
                plt.scatter(0,15) 
                plt.scatter(self.ptx,self.pty)
                plt.pause(0.1)     


if(__name__=="__main__"):
    movement_detect = movement_detector()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        movement_detect.main()
        rate.sleep()

    plt.show()