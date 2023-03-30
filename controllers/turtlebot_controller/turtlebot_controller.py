"""move_to_position controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
#from controller import Robot
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
    return angle_trunc(atan2(abs(deltaY), abs(deltaX)))


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
    max_speed = 6.67
   
    # enable motors
   
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    #print("left motor",left_motor)
    #print("right motor",right_motor)
   
    right_sensor = robot.getDevice('right wheel sensor')
    right_sensor.enable(timestep)

   
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
   
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
   
   
    lidar_sensor = robot.getDevice("LDS-01")
    #print("lidar_sensor",lidar_sensor)
    lidar_sensor.enable(timestep)
    lidar_sensor.enablePointCloud()
   
   
    #create_node = robot.getDevice("Create")
    #print("create ",create_node)
    # trans_field = position_sensor.getField("translation")
    # values = trans_field.getSFVec3f()
    # print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))
   
   
   
    # gps = robot.getDevice("gps")
    # print("gps",gps)
    # gps.enable(timestep)
   
    compass = robot.getDevice("compass")
    compass.enable(timestep)
   
    print("compass",compass)
   
    # goal_x = goal_x
    # goal_y = goal_y
   
   
    angles = []
    left_speed = max_speed
    right_speed = max_speed
    # Robots position
    robot_node = robot.getFromDef("TurtleBot3Burger")
    obs1_node = robot.getFromDef("Create0")
    obs2_node = robot.getFromDef("Create1")
    if robot_node is None:
        sys.stderr.write("No DEF TurtleBot node found in the current world file\n")
        sys.exit(1)
    trans_field_turtle = robot_node.getField("translation")
    trans_field_obs1 = obs1_node.getField("translation")
    trans_field_obs2 = obs2_node.getField("translation")
    #init_state = "to_goal"
    while robot.step(timestep) != -1:
        compass_value = compass.getValues()
        bearing_in_degrees= get_bearing_in_degrees(compass_value)
        bearing_in_degrees = 360-bearing_in_degrees
        #print("heading angle",bearing_in_degrees)
       
        # Checking robot position
        values_turtle = trans_field_turtle.getSFVec3f()
        values_obs1 = trans_field_obs1.getSFVec3f()
        values_obs2 = trans_field_obs2.getSFVec3f()
        # print("Turtle is at position: %g %g" % (values_turtle[0], values_turtle[1]))
        # print("obs1 is at position: %g %g" % (values_obs1[0], values_obs1[1]))
        # print("obs2 is at position: %g %g" % (values_obs2[0], values_obs2[1]))
       
        #print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))
        #gps_val = gps.getValues()
        #print("robot location:",float("{:.2f}".format(gps_val[0])),float("{:.1f}".format(gps_val[1])))
       
        range_image = lidar_sensor.getRangeImage()
        lidar1, lidar2, lidar3, lidar4, lidar5 = range_image[0], range_image[91], range_image[180], range_image[280], range_image[44]
        print("lidar range",lidar1, lidar2, lidar3, lidar4, lidar5)
        #print("length of lidar data",len(range_image))
        #print(lidar1)
        #print(lidar2, lidar1, bearing_in_degrees)
        #if init_state == "to_goal":
           
             
        if lidar1 < 0.2 and lidar2 > 0.4 and bearing_in_degrees >= 90:
            left_speed = max_speed
            right_speed = -max_speed
       
        elif lidar1 < 0.20 and bearing_in_degrees < 90:
            left_speed = -max_speed
            right_speed = max_speed
        else:
            if lidar5 > 1000:
                left_speed = -max_speed
                right_speed = max_speed
            else:
                left_speed = max_speed
                right_speed = max_speed
       
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
       

if __name__=="__main__":
    my_robot = Supervisor()
    run_robot(my_robot)
