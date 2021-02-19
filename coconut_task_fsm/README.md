This package is a finite state machine(fsm) for coconut.
This state machine is for moving autonomously from goal to goal given a list of goals.

    states in coconut_task_fsm.py
    
    0.idle state, waiting for command
        received "start" from topic "gui_cmd": proceed to 1
    1.rotate to localize self
    2.pop goal from list goals(first in first out) and navigate to that goal.
        success: proceed to 3
        failed: terminated
    3.perform task, wait user input
        goals list empty: proceed to 4
        goals list not empty: repeat from state 2 
    4.return to position with name "base_station"
        success: terminated
        failed: terminated


How to use?

overall
    1. Run call_launch.launch after turn on robot.
    2. Run backend. (See coconut_backend for example)
    3. Set rosparam "/map_name".
    4. Publish topic "/call_launch" with data="robot". This will launch coconut_robot/coconut_robot.launch
    5. Save tasks to coconut_task_fsm/task/task.txt
    6. Publish topic "/gui_cmd" with data="start". This will run coconut_task_fsm.py
