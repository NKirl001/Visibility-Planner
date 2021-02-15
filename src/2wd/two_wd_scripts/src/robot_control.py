#!/usr/bin/env python

#ROS Stuf
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations

#Python Classes
import time
import math

#Custom Classes
from Node import Node
from PID import PID

rad_to_deg = 180/3.14

class RobotControl():

    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/2wd/cmd_vel', Twist, queue_size=1)
        self.laser_subscriber = rospy.Subscriber(
            '/2wd/laser/scan', LaserScan, self.laser_callback)
        self.odom_subcriber =rospy.Subscriber(
            '/2wd/odom', Odometry, self.odom_callback)     
        self.odom_msg = Odometry()
        self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)
        
        #temp poss===========================================
        self.dist = 0
        self.time = 0
        self.tolerance = 0.3
        
        #Custom stuff
        self.pid = PID()

       

    def publish_once_in_cmd_vel(self):
        
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
                
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
        
    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()
        
    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_front_laser(self):
        time.sleep(1)
        return self.laser_msg.ranges[360]
        
    def get_laser_regions(self):
        time.sleep(1)
        self.regions = { '0': min(self.laser_msg.ranges[0],10), # right 
                         '45':    min(self.laser_msg.ranges[180],10), #front right
                         '90': min(self.laser_msg.ranges[360],10), #front
                         '135':   min(self.laser_msg.ranges[540],10), #front left
                         '180':  min(self.laser_msg.ranges[719],10), #left
                       }
                       
        
        return self.regions

    def get_laser_full(self):
        time.sleep(1)
        return self.laser_msg.ranges
        
    def odom_callback(self, msg_):
        self.odom_msg = msg_
        
    def getcurrentposition(self):
        self.position_x = round(self.odom_msg.pose.pose.position.x, 3)
        self.position_y = round(self.odom_msg.pose.pose.position.y, 3)
        return  self.position_x, self.position_y
        
    def getcurrentyaw(self):
        self.quaternion = (
                      self.odom_msg.pose.pose.orientation.x,
                      self.odom_msg.pose.pose.orientation.y,
                      self.odom_msg.pose.pose.orientation.z,
                      self.odom_msg.pose.pose.orientation.w)
        self.euler = transformations.euler_from_quaternion(self.quaternion)
        self.yaw_ = self.euler[2]
        return round(self.yaw_ , 3)#* rad_to_deg #self.yaw_  this is in radians
       
    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
            self.cmd.linear.x = speed
        elif motion == "backward":
            self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s


    
#===============================specifics=========================================================        

    def rotodom(self, direction):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0
        
        target = int(direction)
        
         
        if target > 90:
          self.cmd.angular.z = -0.3
        elif target < 90:
          self.cmd.angular.z = 0.3
        
           
        if self.cmd.angular.z != 0: 
            ang_tol = 0.05
            io = 0
            final_ = abs((target - 90) * (math.pi/180))
            
            time = final_ / abs(self.cmd.angular.z)
            count = 0
            while (count <= time):
                count += 1
                
                # Publish the velocity
                self.vel_publisher.publish(self.cmd)
                self.rate.sleep()

            # set velocity to zero to stop the robot
            self.stop_robot()
            
                    
    def move_straight_distance(self, start_node, end_node):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0
        self.cmd.linear.x = 0.5
        
        self.temp_start = start_node
        self.temp_goal = end_node
  
        self.dist = math.sqrt(pow(abs(self.temp_start.x - self.temp_goal.x),2) + pow(abs(self.temp_start.y - self.temp_goal.y),2))
        self.time = self.dist/self.cmd.linear.x
        
        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        early_stopping = False
        while (i <= self.time or early_stopping == False):
            
            if self.regions['90'] <= 5:
              early_stopping = True
              print("Early Stopping Active")
              print('\n')
            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            i += 1
            self.rate.sleep()
            

              

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + str(round(self.dist,3)) + " for " + str(self.time) + " seconds"
        print( s)
    
    def getcurrentnode(self):    
        self.tempNode = Node()
        self.tempNode.setcoord(self.getcurrentposition())
    

    #Current Best Movement      
    def move_PID(self, start_node, goal_node):
        
        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0
        
        while(self.pid.done != True):
          self.getcurrentnode()
          self.pid.one_shot(self.tempNode, goal_node)
          self.cmd.linear.x = self.pid.velocity_out 
          self.vel_publisher.publish(self.cmd)
         
          if self.get_front_laser() <= 5: # change value for early stop best: 5
            self.pid.done = True
            print("Early Stopping Active")          
          
          self.rate.sleep()
          
        self.stop_robot()
        self.pid.reset() 
               
    def move_check(self):
        # Initilize velocities
        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0
        
        self.get_laser_regions()
        while (self.regions["0"] < 2 and self.regions["90"] > 1.5):
          self.get_laser_regions()
          # Publish the velocity
          self.publish_once_in_cmd_vel()
          self.rate.sleep()
        
        self.stop_robot()

        
              


