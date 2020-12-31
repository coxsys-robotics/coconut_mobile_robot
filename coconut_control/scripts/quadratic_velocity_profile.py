#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


"""
Subscribe velocity from twist_mux and apply velocity profile.
Use S-curve velocity profile.
Calculate velocity curve using quadratic formula.
"""


twist = Twist()
eiei = Float32() ### msg for debug

### get velocity limit from rosparam
max_linear_vel = rospy.get_param("/config/limit_lin_vel", 0.5)
max_angular_vel = rospy.get_param("/config/limit_ang_vel", 0.5)

### get acceleration limit from rosparam
max_linear_acc = rospy.get_param("/config/limit_lin_acc", 1.5)
max_angular_acc = rospy.get_param("/config/limit_ang_acc", 2.0)

acc_max = [max_linear_acc, 1.0, max_angular_acc] ### [max_acc_x, max_acc_y, max_acc_angular]

#index list
linear_x = 0
linear_y = 1
angular_z = 2

### joystick controller dead zone
threshold = [0.064, 0.013, 0.001]   #linear_x, linear_y, angular_z

#vel target
cmd_vel = [0, 0, 0]                 #linear_x, linear_y, angular_z
#pre vel target
pre_value = [0, 0, 0]               #linear_x, linear_y, angular_z

vel_now = [0, 0, 0]                 #linear_x, linear_y, angular_z

acc_now = [0, 0, 0]                 #linear_x, linear_y, angular_z
#tau = time now - current time stamp
tau_use = [0, 0, 0]                 #linear_x, linear_y, angular_z
#time
T = [0, 0, 0]                       #linear_x, linear_y, angular_z

#coef variable
c_value = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]

Hz_command = 50

joy_emer_stop = False

"""
scale value from 1 unit to vel
data : value from axes joy
threshould : dead zone
"""
def cal_vel(data, threshould):
    if data > threshould:
        vel = (data - threshould) / (1.0 - threshould)
    elif data < ((-1) * threshould):
        vel = (data + threshould) / (1.0 - threshould)
    else:
        vel = 0
    return vel

"""
find coef
vi = vel initial
ai = acc initial
vf = vel target
af = acc target (0)
T = time of traj
"""
def coef(vi, ai, vf, af, T):
    c0 = vi
    c1 = ai
    c2 = ((3*(vf-vi))/(T**2))-((af+(2*ai))/T)
    c3 = -((2*(vf-vi))/(T**3))+((af+ai)/(T**2))
    return [c0, c1, c2, c3]

def emer_stop(data):
    global joy_emer_stop
    if data.buttons[5]:
        for axis in range(3):
            pre_value[axis] = 0
            cmd_vel[axis] = 0
            tau_use[axis] = 999
        joy_emer_stop = True
    else:
        joy_emer_stop = False

def callback(data = Twist()):
    global max_linear_vel, max_angular_vel
    global cmd_vel, pre_value, vel_now, acc_now, tau_use, T
    global joy_emer_stop
    # axes = list(data.axes)
    # if data.buttons[0]:
    #     max_linear_vel = max_linear_vel - 0.1
    #     if max_linear_vel < 0.1:
    #         max_linear_vel = 0.1
    # if data.buttons[3]:
    #     max_linear_vel = max_linear_vel + 0.1
    #     if max_linear_vel > 0.5:
    #         max_linear_vel = 0.5
    # if data.buttons[2]:
    #     max_angular_vel = max_angular_vel - 0.1
    # if data.buttons[1]:
    #     max_angular_vel = max_angular_vel + 0.1
    # if data.buttons[4]:
    #     for axis in range(3):
    #         pre_value[axis] = 0
    #         cmd_vel[axis] = 0
    #         tau_use[axis] = 999
    #     return
    
    """
    cal target vel from axes data
    """
    # cmd_vel[linear_x] = cal_vel(axes[1], threshold[linear_x]) * max_linear_vel
    # cmd_vel[linear_y] = cal_vel(axes[0], threshold[linear_y]) * max_linear_vel
    # cmd_vel[angular_z] = cal_vel(axes[3], threshold[angular_z]) * max_angular_vel
    if(joy_emer_stop):
        return
    cmd_vel = [data.linear.x, data.linear.y, data.angular.z]

    for axis in range(3):
        if cmd_vel[axis] != pre_value[axis]:
            pre_value[axis] = cmd_vel[axis]
            T[axis] = 0.1
            """
            loop increase time(T) for check limit acc
            """
            exitt = True
            while exitt:
                exitt = False
                c_value[axis] = coef(vel_now[axis], acc_now[axis], cmd_vel[axis], 0, T[axis])
                for i in range(int(T[axis]*Hz_command)):
                    tau = i*(1/Hz_command)
                    acc = c_value[axis][1] + (2*c_value[axis][2]*tau) + (3*c_value[axis][3]*(tau**2))
                    if abs(acc) > acc_max[axis]:
                        T[axis] = T[axis] + 0.1
                        exitt = True
                        break
            tau_use[axis] = 0

if __name__ == '__main__':
    rospy.init_node('joy_control', anonymous=True)
    pub = rospy.Publisher('/coconut_vel', Twist, queue_size=10)
    pub_acc = rospy.Publisher('/acc', Float32, queue_size=10)
    rospy.Subscriber('/joy', Joy, emer_stop)
    rospy.Subscriber('/twist_mux/cmd_vel', Twist, callback)

    r = rospy.Rate(Hz_command)
    while(not rospy.is_shutdown()):
        for axis in range(3):
            if tau_use[axis] < T[axis]:
                vel_now[axis] = c_value[axis][0] + (c_value[axis][1]*tau_use[axis]) + (c_value[axis][2]*(tau_use[axis]**2)) + (c_value[axis][3]*(tau_use[axis]**3))
                acc_now[axis] = c_value[axis][1] + (2*c_value[axis][2]*tau_use[axis]) + (3*c_value[axis][3]*(tau_use[axis]**2))
            else:
                vel_now[axis] = cmd_vel[axis]
                acc_now[axis] = 0
            tau_use[axis] = tau_use[axis]+(1/Hz_command)
        twist.linear.x = vel_now[linear_x]
        twist.linear.y = vel_now[linear_y]
        twist.angular.z = vel_now[angular_z]
        pub.publish(twist)
        r.sleep()