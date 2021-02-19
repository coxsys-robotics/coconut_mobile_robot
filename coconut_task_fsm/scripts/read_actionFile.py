#!/usr/bin/env python3

# Import necessary package
from findCurrent_angle import find_currentAngle

# Select action for the robot 
def read_action(target, state):
    turnDegree_currentRoom = 0
    Degree1 = 0
    Degree2 = 0
    
    if state == "turn2currentRoom":
        degree1, degree2, turnDegree_currentRoom = find_currentAngle(target)
        read_action.rotate2currentRoom = turnDegree_currentRoom
        read_action.Degree1 = degree1
        read_action.Degree2 = degree2
