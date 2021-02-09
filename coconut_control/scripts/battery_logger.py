import os
import time
import rospy
from std_msgs.msg import UInt8, Float32
from geometry_msgs.msg import Twist

"""
    Log battery voltage level 
    Log file format:
        text file with extension as .log with start time as file name 
            YYYY-MM-DD-h-m-s.log where Y:year, M:month, D:date, h:hour, m:minute, s:second
        every line will be --> <time stamp>, <data status>, <battery voltage>
        data status is something like "info", "error", etc...
"""
class battery_logger():
    def __init__(self, save_directory="Battery_Log", log_every_n_second=15):
        self.save_directory = os.path.expanduser("~") + '/' + save_directory
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
        t = time.localtime()
        self.Log_filename = str(t.tm_year) + "-" + str(t.tm_mon) + "-" + str(t.tm_mday) + "-" + str(t.tm_hour) + "-" +str(t.tm_min) + "-" +str(t.tm_sec) 

        #rospy.init_node("battery_logger")

        if log_every_n_second <= 0:
            log_every_n_second = 60
        self.rate = rospy.Rate(1.0/log_every_n_second)

        ### PERCENT ###
        self.new_percent = False
        self.percent = UInt8()
        self.percent_sub = rospy.Subscriber("/battery_percent", UInt8, self.percent_callback)

        ### VOLTAGE ###
        self.new_volt = False
        self.volt = Float32()
        self.volt_sub = rospy.Subscriber("/sens_voltage", Float32, self.volt_callback)

        ### CURRENT(Amp) ###
        self.new_amp = False
        self.amp = Float32()
        self.amp_sub = rospy.Subscriber("/sens_current", Float32, self.amp_callback)

        ### STATUS ###
        self.new_status = False
        self.status = ""
        self.bat_status_sub = rospy.Subscriber("/battery_status", UInt8, self.bat_stat_callback)

    def bat_stat_callback(self,data):
        """
        battery status ids and their meaning.
        """
        if(data.data==0):
            self.status = "normal"
        elif(data.data==1):
            self.status = "charging"
        elif(data.data==2):
            self.status = "charged"
        elif(data.data==3):
            self.status = "error"
        self.new_status = True

    def percent_callback(self, data):
        self.percent = data
        self.new_percent = True

    def volt_callback(self, data):
        self.volt = data
        self.new_volt = True

    def amp_callback(self, data):
        self.amp = data
        self.new_amp = True
        

    def log_to_file(self):
        """
        Write a new line to log file after received all required info.
        """
        print("log data to: " + self.save_directory + '/' + self.Log_filename + '.csv')
        bat_percent = str(self.percent.data)
        bat_volt = str(self.volt.data)
        bat_amp = str(self.amp.data)
        if( not self.new_status or not self.new_percent or not self.new_volt or not self.new_amp):
            self.status = "error"
            bat_percent = 0
            bat_volt = 0
            bat_amp = 0
        with open(self.save_directory + "/" + self.Log_filename + ".csv", "a") as logfile:
            t = time.localtime()
            time_str = str(t.tm_hour) + "-" +str(t.tm_min) + "-" +str(t.tm_sec) 
            line = time_str + "," + self.status + "," + str(bat_volt) + "," + str(bat_amp) + "," + str(bat_percent) + "\n"
            logfile.write(line)
        self.new_status = False
        self.new_volt = False
        self.new_percent = False
        self.new_amp = False

if __name__=="__main__":
    Logger = battery_logger()
    try:
        while not rospy.is_shutdown():
            Logger.rate.sleep()
            Logger.log_to_file()
    except KeyboardInterrupt:
        rospy.signal_shutdown("shutdown")
