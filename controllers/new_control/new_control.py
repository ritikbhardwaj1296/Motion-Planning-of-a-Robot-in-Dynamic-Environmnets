"""move_to_position controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor
from controller import Supervisor
import sys
import math

from math import *

def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return a

def getAngleBetweenPoints(x_orig, y_orig, x_landmark, y_landmark):
    deltaY = y_landmark - y_orig
    deltaX = x_landmark - x_orig
    return angle_trunc(atan2(deltaY, deltaX))


def get_distance(goal_x,goal_y,x_current,y_current):
    return ((goal_y-y_current)**2+(goal_x-x_current)**2)**(1/2)
    
def get_bearing_in_degrees(north):
  rad = math.atan2(north[1], north[0])
  bearing = (rad - 1.5708) / 3.14 * 180.0
  if (bearing < 0.0):
    bearing = bearing+360
    
  return bearing

def run_robot(robot):

    
    
    TIME_STEP = 32

    timestep =1
    max_speed =6.67
    # enable motors
    
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    print("left motor",left_motor)
    print("right motor",right_motor)
    
    right_sensor = robot.getDevice('right wheel sensor')
    right_sensor.enable(timestep)

    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    
    lidar_sensor = robot.getDevice("LDS-01")
    print("lidar_sensor",lidar_sensor)
    lidar_sensor.enable(timestep)
    lidar_sensor.enablePointCloud()
    
    
    # create_node = robot.getFromDef("Create")
    # print("create ",create_node)
    # trans_field = position_sensor.getField("translation")
    # values = trans_field.getSFVec3f()
    # print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))
    
    
    
    #gps = robot.getDevice("gps")
    #print("gps",gps)
    #gps.enable(timestep)
    
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    
    robot_node = robot.getFromDef("Turtle")
    obs1_node = robot.getFromDef("obs1")
    obs2_node = robot.getFromDef("obs2")
    if robot_node is None:
        sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
        sys.exit(1)
    trans_field_turtle = robot_node.getField("translation")
    trans_field_obs1 = obs1_node.getField("translation")
    trans_field_obs2 = obs2_node.getField("translation")
    #print("compass",compass)
    
    # goal_x = goal_x
    # goal_y = goal_y
    
    
    angles = []
    left_speed = max_speed
    right_speed = max_speed
    prev_obs1 = []
    prev_obs2 = []
    cnt = 0
    while robot.step(timestep) != -1:
        #print(robot.step(timestep))
        compass_value = compass.getValues()
        bearing_in_degrees= get_bearing_in_degrees(compass_value)
        bearing_in_degrees = 360-bearing_in_degrees
        print("bearing angle", bearing_in_degrees)
        values_turtle = trans_field_turtle.getSFVec3f()
        values_obs1 = trans_field_obs1.getSFVec3f()
        values_obs2 = trans_field_obs2.getSFVec3f()
        
        if cnt == 0:
            prev_obs1.append((values_obs1[0], values_obs1[1]))
            prev_obs1.append((values_obs1[0], values_obs1[1]))
        else:
            prev_obs1[0] = prev_obs1[1]
            prev_obs1[1] = (values_obs1[0], values_obs1[1])
            
        if cnt == 0:
            prev_obs2.append((values_obs2[0], values_obs2[1]))
            prev_obs2.append((values_obs2[0], values_obs2[1]))
        else:
            prev_obs2[0] = prev_obs2[1]
            prev_obs2[1] = (values_obs2[0], values_obs2[1])
            
        h_angle_obs1 = getAngleBetweenPoints(prev_obs1[0][0], prev_obs1[0][1], prev_obs1[1][0], prev_obs1[1][1])
        h_angle_obs1 = math.degrees(h_angle_obs1)
        print(h_angle_obs1)
        h_angle_obs2 = getAngleBetweenPoints(prev_obs2[0][0], prev_obs2[0][1], prev_obs2[1][0], prev_obs2[1][1])
        h_angle_obs2 = math.degrees(h_angle_obs2)
        #gps_val = gps.getValues()
        gps_x = values_turtle[0]
        gps_y = values_turtle[1]
        cnt += 1
        dist_obs1 = get_distance(gps_x,gps_y,values_obs1[0],values_obs1[1])
        dist_obs2 = get_distance(gps_x,gps_y,values_obs2[0],values_obs2[1])
        print("obs1 distance", dist_obs1)
        print("obs2 distance", dist_obs2)
       
        
        point1_x = -0.713
        point1_y = -1.12
        
        point2_x= -0.168
        point2_y = -1.11
        
        point3_x = -0.186
        point3_y = 1.4
        
        point4_x = 0.155
        point4_y = 1.345
        
        point5_x = 0.12
        point5_y = -1.23
        
        point6_x = 0.7
        point6_y = -1.23
        range_image = lidar_sensor.getRangeImage()
        
        print("Distance from goal", get_distance(point6_x,point6_y,gps_x,gps_y))
        angl = getAngleBetweenPoints(gps_x, gps_y, point6_x, point6_y)
        angl = math.degrees(angl)
        difference = bearing_in_degrees - angl
        print("difference", difference)
        
        point_4_distance = get_distance(point4_x,point4_y,gps_x,gps_y)
        point_5_distance = get_distance(point5_x,point5_y,gps_x,gps_y)
        
        if gps_x < 0 and dist_obs1 < 0.3 and (h_angle_obs1 < 90 or h_angle_obs1 > 270):
            if h_angle_obs1 < 360 or h_angle_obs1 > 270:
                left_speed = -max_speed
                right_speed = -max_speed
                print("I'm going back...from obs1")
            else:
                left_speed = max_speed
                right_speed = max_speed
        elif gps_x > 0 and dist_obs2 < 0.3 and (h_angle_obs2 > 90 or h_angle_obs2 < 270):
            if h_angle_obs2 < 180 or h_angle_obs2 > 90:
                left_speed = -max_speed
                right_speed = -max_speed
                print("I'm going back...from obs2")
            else:
                left_speed = max_speed
                right_speed = max_speed            
        elif get_distance(point1_x,point1_y,gps_x,gps_y)<0.05 and bearing_in_degrees>1:
            print("robot at point 1")
            left_speed = max_speed*(1/6)
            right_speed = max_speed*(1/6)
        elif round(gps_x, 2) == -0.16 and round(gps_y, 2) < 0 and bearing_in_degrees<90:
            print("robot at point 2")
            left_speed = -max_speed*(1/6)
            right_speed = max_speed*(1/6)
        elif round(gps_y, 1) == 1.4 and bearing_in_degrees>1 and round(gps_x, 1) < 0.10:
            print("robot at point 3")
            left_speed = max_speed*(1/6)
            right_speed = -max_speed*(1/6)
        elif round(gps_x, 2) == 0.7 and gps_y > 0 and (bearing_in_degrees<1 or bearing_in_degrees>270) :
            print("robot at point 4")
            left_speed = max_speed*(1/6)
            right_speed = -max_speed*(1/6)
        elif round(gps_y, 2) == -1.23 and round(gps_x, 1) > 0 and (bearing_in_degrees>269 or bearing_in_degrees<0.40):
            print("robot at point 5")
            left_speed = -max_speed*(1/6)
            right_speed = max_speed*(1/6)
        elif get_distance(point6_x,point6_y,gps_x,gps_y)<0.05:
            print("robot reached at target point")
            
            left_speed = 0
            right_speed = 0
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            sys.exit(1)
        
        # elif get_distance(point6_x,point6_y,gps_x,gps_y)<0.3:
            # if abs(difference) >2:
                # if difference < 0:
                    # left_speed = max_speed
                    # right_speed = -max_speed
                # else:
                    # left_speed = -max_speed
                    # right_speed = max_speed
            # else :
                # left_speed = max_speed
                # right_speed = max_speed
        
        else :
            left_speed = max_speed
            right_speed = max_speed
            
      
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        print("   ")
        

if __name__=="__main__":
    my_robot = Supervisor()
    run_robot(my_robot)
        
        
