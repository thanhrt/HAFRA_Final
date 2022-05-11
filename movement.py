#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Float32
import moveit_commander
import geometry_msgs.msg
from math import pi, radians
from moveit_commander.conversions import pose_to_list
from ur_msgs.msg import *
from ur_msgs.srv import *

# Create a global list to store x,y coordinates
coordinates = []

#Creates a global variable to hold the subscriber infromation
#This is needed to unsubscribe to not create duplicate subscribers
global sub_once

#Function to turn on the camera
#This is done by choosing a specific IO pin to turn on manually through script
def turn_on_vacuum():
    #rospy service specifically meant for setting the state of an io pin
    #needs to be requested before being able to be utilized
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    io_serv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    
    #fun specifies the specific function that needs to be used
    #pin is the specific pin that the user wants to turn on
    #state is the on or off condition (0 off, 1 on)
    digital_io0_on_request = SetIORequest()
    digital_io0_on_request.fun = 1
    digital_io0_on_request.pin = 0
    digital_io0_on_request.state = 1
    response = io_serv(digital_io0_on_request)
    return response.success


#Same functionality as turning on the vacuum, only state is changed to off
def turn_off_vacuum():
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    io_serv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    digital_io0_off_request = SetIORequest()
    digital_io0_off_request.fun = 1
    digital_io0_off_request.pin = 0
    digital_io0_off_request.state = 0
    response = io_serv(digital_io0_off_request)
    return response.success

#Make a list of user inputs to append to
def store_coordinates(my_data):
    global coordinates
    coordinates.append(my_data)

#Data from the other programs comes in the format "data: ##.##"
#Call the store_coordinate function to append all user entered data
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %f", data.data)
    my_data = data 
    store_coordinates(my_data)
    
#This is where the program waits for information from the other program
def listener():
    
    #Program subscribes to specific node, and utilizes the callback function to 
    #wait for data to continue
    sub_once = rospy.Subscriber('Camera', Float32, callback)
    
    #Global coordinate needs to be declared again inside a functino
    global coordinates
    
    #Waits for atleast 2 values (x and y) before exitting the function
    while (len(coordinates) < 2): # checking if array has 0 or 1 coordinate to call callback function again (to get the y coord)
        rospy.sleep(2)
    #If it has received at least 2 functions, then unsubcribe (so it can avoid duplicate subscribers)
    #and then exit the function
    else:
        sub_once.unregister()
        return

        

#Receives coordinates from vision program, and convert it to float (from string)
def convert():
    #Same case as above
    global coordinates

    #x and y coordinates come in the format of "data: xvalue"
    #So we need to split it using space, and then only grab the numerical value
    x_pos = str(coordinates[0])
    x_string = x_pos.split(' ')[1]
    
    y_pos = str(coordinates[1])
    y_string = y_pos.split(' ')[1]
    
    #The value comes in a string format, convert it to float
    x_pos_num = float(x_string)
    y_pos_num = float(y_string)
    
    #If the values received for x and y are both 0, then exit the program
    #as this is the special case for exit
    if(x_pos_num == 0 and y_pos_num == 0):
        print('Exit Code Received')
        exit()

    #Return x and y values needed for the robot to know where to go
    return x_pos_num, y_pos_num

if __name__ == '__main__':
    #Move it objects that has many useful functions like moving the robot based on 
    #joint goals or xyz coordinates
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")



    #Using inverse kinematics, these are the specific joint angles needed to be in the
    #specified "start state"
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = radians(180)
    joint_goal[1] = radians(-125)
    joint_goal[2] = radians(-55)
    joint_goal[3] = radians(-90)
    joint_goal[4] = radians(90)
    joint_goal[5] = 0
    

    # orientations for the new pick-up state
    pose_goal = geometry_msgs.msg.Pose()
    #pose_goal.orientation.w = 1.38089e-05
    pose_goal.orientation.x = -0.70710
    pose_goal.orientation.y = 0.70711
    pose_goal.orientation.z = -9.8189e-06
    
    # drop-off state (at bin)
    joint_goal3 = group.get_current_joint_values()
    joint_goal3[0] = radians(90)
    joint_goal3[1] = radians(-96)
    joint_goal3[2] = radians(-82)
    joint_goal3[3] = radians(-92)
    joint_goal3[4] = radians(90)
    joint_goal3[5] = 0


    #Create a listener node to listen for data from the talker
    rospy.init_node('listener', anonymous=True)
    #Infinite loop, can only be exitted out manually by the user by entering
    #in "0 0"
    while(1):
        print("Calling listener...")
        
        #Calls the listener function
        listener()
        
        #Gets the x and y values from the talker, and converts it into a float
        rospy.sleep(2)
        x_goal, y_goal = convert()


        #This is put into the coordinate pose variable for rospy's movement function
        pose_goal.position.x = x_goal
        pose_goal.position.y = y_goal
        pose_goal.position.z = 0.1400

        #Gives time for path finding
        group.set_planning_time(10.0)
        
        #Assuming that it's already at start state
        
        #Go to the either the detected marker, or to the position given by the user
        #Turn on the vacuum and then wait 2 seconds before going on
        group.go(pose_goal, wait=True)
        turn_on_vacuum()
        rospy.sleep(2)
        
        #Go to the drop off location, after getting there turn off vacuum to drop off the
        #item, and then wait for 1 second
        group.go(joint_goal3, wait=True)
        turn_off_vacuum()
        rospy.sleep(1)
        
        #Go back to the start state to repeat, and clear coordinate variable
        group.go(joint_goal, wait=True)
        coordinates = []

    
    group.stop
    group.clear_pose_targets()


