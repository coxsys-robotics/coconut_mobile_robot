This package is for using coconut_agv with another application

launch call_launch.launch will run various python scripts.
each scripts will subscribe to "/call_launch" topic (message type: String)
and will launch another roslaunch when receive specific command.
e.g. call_online_slam.py will launch online slam when receive command "online_slam" from topic

list of command:
    bringup : launch bringup.launch 
    bringup_shutdown : shutdown bringup.launch
    filter_bag : filter rosbag for offline slam
    loop_save_map : launch loop_save_map.launch to keep saving map while performing slam to sent and display on app
    loop_save_map_shutdown : shutdown loop_save_map.launch, use when terminate slam
    offline_slam : launch offline_slam, must have .bag file from online slam first
    online_slam : launch online_slam, perform mapping
    robot : launch robot.launch, ready to receive tasks
    robot_shutdown : shutdown robot.launch. not power-off robot (maybe need to change name soon)
    save_map : save map after finish slam
    
