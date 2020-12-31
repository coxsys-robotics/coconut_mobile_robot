#!/usr/bin/env python3

import rospy
import random
import time
from std_msgs.msg import UInt8, UInt8MultiArray, UInt16, UInt16MultiArray
from sensor_msgs.msg import Joy


class neopixel_test:
    def __init__(self):
        ### Neopixel (read manual at https://docs.google.com/spreadsheets/d/1sDBpzWSkR-TJ_CMlEihfyFtKzg1GNOJjjiK5VN6d-MU/edit#gid=0) ###
        ## neopixel mode ##
        self.neopixel_mode_msg = UInt8()
        self.neopixel_mode_publisher = rospy.Publisher("neopixel_mode", UInt8, queue_size=10)
        ## neopixel rgb : [r0,g0,b0, r1,g1,b1, r2,g2,b2] ##
        self.neopixel_rgb_msg = UInt8MultiArray()
        self.neopixel_rgb_publisher = rospy.Publisher("neopixel_rgb", UInt8MultiArray, queue_size=10)
        ## neopixel time ##
        self.neopixel_time_msg = UInt16MultiArray()
        self.neopixel_time_publisher = rospy.Publisher("neopixel_time_ms", UInt16MultiArray, queue_size=10)
        ## neopixel number of led ##
        self.neopixel_num_msg = UInt16MultiArray()
        self.neopixel_num_publisher = rospy.Publisher("neopixel_num", UInt16MultiArray, queue_size=10)
        ## neopixel number of step in run mode ##
        self.neopixel_step_msg = UInt16()
        self.neopixel_step_publisher = rospy.Publisher("neopixel_step", UInt16, queue_size=10)
        ### ### ###

        self.joy_sub = None

        self.neopixel_mode = 0
        self.r = 20
        self.g = 40
        self.b = 90
        
        self.prv_up = 0
        self.prv_down = 0
        self.prv_left = 0
        self.prv_right = 0

        self.charge_percent = 0
        self.charge_mode = False
        self.low_battery_mode = False

        self.wait_mode_change = False

    def start(self):
        rospy.init_node('test_neopixel')
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        time.sleep(1)
        self.neopixel_command(0, self.r, self.g, self.b)
        self.neopixel_command(0, self.r, self.g, self.b)
        

    def joy_callback(self, data):
        """
        arrow up : buttons[-2]   -- charge percent up by 5
        arrow down : buttons[-1]  -- charge percent down by 5
        arrow left : buttons[-4]  -- random color mode 0
        arrow right : buttons[-3]  -- toggle low battery aleart mode
        lt : axes[2]  -- reset percent and mode
        rt : axes[4]  -- 
        """

        ## lt
        if(data.axes[2]==-1):
            self.reset()
        ## arrow right
        elif(data.buttons[-3] and not self.prv_right):
            if(self.low_battery_mode):
                self.charge_mode = True
                self.r = 100
                self.g = self.b = 0
                # self.neopixel_command(255, 0,0,0)
                # self.neopixel_command(255, 0,0,0)
                # time.sleep(1)
            else:
                self.neopixel_command(8, 20,0,0,1)
                self.neopixel_command(8, 20,0,0,1)
            self.low_battery_mode = not self.low_battery_mode
        ## arrow left
        elif(data.buttons[-4]  and not self.prv_left):
            self.random_color()

        if(self.charge_mode):
            ## arrow up
            if(data.buttons[-2] and not self.prv_up):
                self.charge_percent += 4 
            ## arrow down
            elif(data.buttons[-1] and not self.prv_down):
                self.charge_percent -= 4
            
            self.charge_percent = min(100, max(self.charge_percent, 0) )

            if(self.charge_percent <= 50):
                self.r = 100
                self.g = min(100, self.charge_percent * 2)
            elif(self.charge_percent > 50 and self.charge_percent <100):
                self.g = 100
                self.r = max(0, 200 - (self.charge_percent * 2))
            elif(self.charge_percent==100):
                self.r = 20
                self.g = 40
                self.b = 90
                self.charge_percent = 0
                self.charge_mode = False
                self.neopixel_command(0, self.r, self.g, self.b)

            self.neopixel_command(0, self.r, self.g, self.b)
        
        self.prv_up = data.buttons[-2]
        self.prv_down = data.buttons[-1]
        self.prv_left = data.buttons[-4]
        self.prv_right = data.buttons[-3]
        

    def reset(self):
        self.charge_mode = False
        self.charge_percent = 0 
        self.neopixel_mode = 0
        self.r = 20
        self.g = 40
        self.b = 90
        self.neopixel_command(0, self.r, self.g, self.b)
        self.neopixel_command(0, self.r, self.g, self.b)
    
    def random_color(self):
        self.r = random.randrange(10,100)
        self.g = random.randrange(10,100)
        self.b = random.randrange(10,100)
        print("{}, {}, {}".format(self.r,self.g,self.b))
        self.neopixel_command(0, self.r, self.g, self.b)

    def neopixel_command(self, mode=0, r=100, g=100, b=100, wait_time=0):
        self.neopixel_rgb_msg.data = [ r, g, b]
        self.neopixel_rgb_publisher.publish(self.neopixel_rgb_msg)
        if(not wait_time):
            time.sleep(wait_time)
        self.neopixel_mode_msg.data = mode
        self.neopixel_mode_publisher.publish(self.neopixel_mode_msg)
            



if(__name__=="__main__"):
    obj = neopixel_test()

    obj.start()
    rospy.spin()

