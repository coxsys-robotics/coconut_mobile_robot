This package is for using coconut_agv with another application

call_launch.launch will run various python scripts.
Each script will subscribe to "/call_launch" topic (message type: String)
and will launch another roslaunch when receive specific command.
e.g. call_online_slam.py will launch online slam when receive command "online_slam" from topic.

List of commands:
    - bringup : launch bringup.launch 
    - bringup_shutdown : shutdown bringup.launch
    - online_slam : launch online_slam, perform mapping
    - online_slam_shutdown : shutdown online_slam, this command doesn't save map. for saving map, use "save_map" command first
    - filter_bag : filter rosbag for offline slam
    - offline_slam : launch offline_slam, must have .bag file from online slam first
    - offline_slam_shutdown : shutdown offline_slam, this command doesn't save map. for saving map, use "save_map" command first
    - save_map : save map for further use. this command doesn't shutdown slam node. use other command to shutdown slam properly
    - loop_save_map : launch loop_save_map.launch, keep saving map image while performing slam for display
    - loop_save_map_shutdown : shutdown loop_save_map.launch
    - robot : launch robot.launch, ready to receive tasks
    - robot_shutdown : shutdown robot.launch. not power-off robot (need to change command name)
    

Currently coconut will launch call_launch.launch using python scripts on robot startup.
See Startup.sh for more info.