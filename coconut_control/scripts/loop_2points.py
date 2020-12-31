#!/usr/bin/env python3

import rospy
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import PoseStamped
import time


##TODO: make it easier to add/remove goal in real time after run program. 

"""
make robot navigation looping between goal_1 and goal_2.
flow:
    <robot_position> --> <goal_2> 
"""

"""
This class will accept list of goal(s) and go to all goal(s) in list.
"""
class Goal_command():
    """
    params:
        goal_list : list of goal(s).
        loop_goal : if set to true, will loop to first goal in list after done last goal.
        wait_between_goal : if true, will wait a for <wait_time> second before go to next goal.
        wait_time : use with wait_between_goal. 
    """
    def __init__(self, goal_list=[], loop_goal=True, wait_between_goal=False, wait_time=2):
        self.goals_list = goal_list
        self.loop_goal = loop_goal

        self.started = False

        self.goal_sent = False
        self.goal_reach = False
        self.goal_abort = False

        self.wait_between_goal = wait_between_goal
        self.wait_time = wait_time

        self.goal_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.goal_callback)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.init_node("goal_loop")

    def add_goal(self, goal):
        self.goals_list.append(goal)

    def remove_goal_at(self, index):
        del self.goals_list[index]

    def clear_goal(self):
        self.goals_list = []

    def goal_callback(self,data):
        """
        callback for movebase goal status
        """
        if self.started:
            if(data.status_list[-1].status==1 and not self.goal_sent):
                print("- goal sent")
                self.goal_sent = True
            elif(data.status_list[-1].status==3 and self.goal_sent):
                print("- goal reach")
                self.started = False
                self.goal_reach = True
            elif(data.status_list[-1].status==2 or data.status_list[-1].status==4 or data.status_list[-1].status==5 or data.status_list[-1].status==8):
                self.goal_abort = True

    def start(self):
        """
        let robot go to all goal in self.goals_list.
        after done, if loop_goal set to True then start all over again.
        if not, break out of loop and stop program.
        """
        # self.started = True
        print(f"goals to go: {len(self.goals_list)}")
        for goal in self.goals_list:
            print("send goal")
            # goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = 'map'
            self.goal_pub.publish(goal)
            self.started = True
            while(not self.goal_reach and not self.goal_abort):
                pass
            if(self.goal_reach):
                self.started = False
                self.goal_sent = False
                self.goal_reach = False
                if(self.wait_between_goal):
                    time.sleep(self.wait_time)
            elif(self.goal_abort):
                print("goal aborted")
                break
        if(self.loop_goal and not self.goal_abort):
            print("---looping goal---")
            self.start()
        else:
            print("---all goals done---")
            self.started = False
            self.clear_goal()

if __name__=="__main__":
    ### goal_1 ###
    point1 = PoseStamped()
    point1.pose.position.x = 0.0
    point1.pose.position.y = 0.0
    point1.pose.orientation.w = 1.0

    ### goal_2 ###
    point2 = PoseStamped()
    point2.pose.position.x = -1.8
    point2.pose.position.y = 0.0
    point2.pose.orientation.w = 1.0

    looper = Goal_command([point2, point1])
    time.sleep(5)
    print("start")
    try:
        looper.start()
        # rospy.spin()  ### looper.start will loop in itself so spin() is not need.
    except KeyboardInterrupt:
        pass
    print("done")
