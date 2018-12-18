#!/usr/bin/env python


import roslib
import rospy
import math
import numpy as np
import geometry_msgs.msg

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

position_of_robot = 0

orientation_of_robot = 0

is_obstacle = False

check_if_point_area = False

is_left_range_data_obstacle = False

ranges = np.zeros((361, 2))	   

def laserscandata(data):
    
    global is_obstacle, is_left_range_data_obstacle,ranges
    range = data.ranges
    range = np.array(range)
    no_of_lines = 0
    maxsize = 100

    for i in np.arange(maxsize):
        if range[180-(maxsize/2)+i] < 1:
            no_of_lines += 1
    if no_of_lines > 1:
        is_obstacle = True
    else:
        is_obstacle = False

    counter = 0
    maxsize = 60
    for i in np.arange(maxsize):
        if range[i] < 1:
            counter += 1
    if counter < 10:
        is_left_range_data_obstacle = False
    else:
        is_left_range_data_obstacle = True
            

def get_position_and_orientation(info):
    
    global position_of_robot, orientation_of_robot
    position_of_robot = info.pose.pose.position
    orientation_of_robot = info.pose.pose.orientation 

def reach_end_point_using_bug2_algo():
	
	velocity_of_robot = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

	current_pose_and_orientation_of_robot = rospy.Subscriber("/base_pose_ground_truth", Odometry, get_position_and_orientation)
	
	        
	rate = rospy.Rate(10)
        
        bot_move_between_states(velocity_of_robot,rate)

	
 
def angle_to_be_turned_when_in_goalseek(goal_angle):
    global is_obstacle,check_if_point_area
    if check_if_point_area:
	return min(goal_angle, 1)
    if is_obstacle:
        return 1
    else:
        return min(goal_angle, 1)
    
def angle_to_turned_when_following_wall():
    global is_left_range_data_obstacle, is_obstacle
    if is_obstacle:
        return 0.5
    
    if is_left_range_data_obstacle:
        return 0
    else:
        return -1 * 0.4

def region_btween_start_end_robot_pos(points):
    

    global position_of_robot
    x1 = points[0,:]
    x2 = points[1,:]
    x3 = np.array([position_of_robot.x, position_of_robot.y])
    area_of_region = abs((x1[0] * (x2[1] - x3[1]) + x2[0] * (x3[1] - x1[1]) + x3[0] * (x1[1] - x2[1])) / 2.0)
    
    threshold = 0.6
    
    if (area_of_region < threshold):
        return True
    else:
	return False


def angle_of_bot(angle):
    return 2 * np.arcsin(angle)	 

    
def bot_move_between_states(velocity_of_robot,rate):
    
    global is_obstacle, orientation_of_robot, position_of_robot, check_if_point_area
    start_end_points = np.array([[-8, -2],[4.5, 9.0]])
    
    endpointreached = False
    
    thresholddist = 0.5
    
    state = "GOAL_SEEK"
    
    while not endpointreached:
           if orientation_of_robot <> 0:
			
			angle_of_robot = angle_of_bot(orientation_of_robot.z)

			current_dist_between_bot_and_end_point = math.sqrt((start_end_points[1, 0] - position_of_robot.x) ** 2 + (start_end_points[1, 1] - position_of_robot.y) ** 2)
            
			angle_between_robot_and_end_point = math.atan((start_end_points[1, 1] - position_of_robot.y) / (start_end_points[1,0]- position_of_robot.x)) - angle_of_robot
            
			check_if_point_area = region_btween_start_end_robot_pos(start_end_points)

			twist = Twist()
		
			if current_dist_between_bot_and_end_point < thresholddist:
 		                    	twist.linear.x = 0 
                			twist.angular.z = 0
					endpointreached = True	
           	        		break
            		else:
                		if is_obstacle:
                    			twist.linear.x = 0.0
                		else:
                    			twist.linear.x = 0.6
                		if state == "GOAL_SEEK":
                    			twist.angular.z =  angle_to_be_turned_when_in_goalseek(angle_between_robot_and_end_point)
                    			if is_obstacle:
                        			state = "WALL_FOLLOW"
                		else:
                    			twist.angular.z = -1 * angle_to_turned_when_following_wall()
                    			if check_if_point_area and not is_obstacle:
                        			state = "GOAL_SEEK"
            		velocity_of_robot.publish(twist)
            		rate.sleep()  
	

if __name__ == '__main__':
    try:
        rospy.init_node('bug2_implementation')
	rospy.Subscriber("/base_scan", LaserScan, laserscandata)
	
        
        reach_end_point_using_bug2_algo()
    except rospy.ROSInterruptException:
        pass


# Inspired by https://github.com/zstring/Ransac-Bug2/blob/master/src/bug2.py
