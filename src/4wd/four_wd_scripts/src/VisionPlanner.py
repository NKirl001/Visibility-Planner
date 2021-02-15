#!/usr/bin/env python

import rospy
from robot_control import RobotControl
from Heap import Heap
from Node import Node
import operator
import time
import math

class PathPlanner(object):

  def __init__(self):
      
      #robotcontrol class
      self.robotcontrol = RobotControl()
      
      #init heap
      self.heap_ = Heap(15)
      self.exploredset = Heap(50)
      self.potentials = Heap(50)
      
      #stores the final list of nodes to travel to
      self.path = []
      
      #stores current neighboring nodes 
      self.neighbornodes = []
      
     
      self.nodecounter = 0
      self.goal = Node()
      self.start = Node()
      
      self.init_ = Node()
      self.init_.visited = 1
      self.init_.setcoord(self.robotcontrol.getcurrentposition())
      self.init_.theta = self.robotcontrol.getcurrentyaw()
      self.setNodeID(self.init_)
      #self.init_.fCost = 0
      self.heap_.add(self.init_)
      #indicates whether node has been visited before, if there is an obstacle or if 
      #unexplored
      #self.explore = 0
     
      print('Robot origin:' ,self.init_)
      print('\n')
     # print('Initial heap: ' , self.heap_.Heap)
      print('\n')
      
      self.planner()
      

  def setNodeID(self, node_):
      
      self.temp = node_ 
      self.temp.NodeID = self.nodecounter
      self.nodecounter += 1
      
  def setgoal(self):
      self.robotcontrol.get_laser_regions()
      self.new_direction = max(self.robotcontrol.regions.iteritems(),
                      key=operator.itemgetter(1))[0]
      self.new_distance = max(self.robotcontrol.regions.iteritems(),
                      key=operator.itemgetter(1))[1]
      
      if self.new_distance < 6: # change distance for recalc best: 6
        print("Searching for new goal")
        print('\n')
        self.robotcontrol.rotodom(-35)
        self.setgoal()

      print('New goal Direction :' , self.new_direction ,'\n New goal Distance: ', self.new_distance)
      print('\n')      
      
      ##Set start node info 
      self.start.setcoord(self.robotcontrol.getcurrentposition())
      self.start.visited = 1 
      self.start.theta = self.robotcontrol.getcurrentyaw()
      self.setNodeID(self.start)
      #self.heap_.add(self.start)
      #self.exploredset.add(self.start)
      #self.start.theta = self.robotcontrol.getcurrentyaw()

      print('Start : ', self.start)
      print('\n') 
      
      self.del_ = self.newcoords(self.new_distance, self.new_direction)
      
      #Set Goal info 
      self.goal.x = round(self.start.x + self.del_[0] , 3)
      self.goal.y = round(self.start.y + self.del_[1], 3)
      #self.goal.theta = self.robotcontrol.getcurrentyaw()
      self.goal.visited = 1
      self.goal.theta = self.start.theta + ((int(self.new_direction)-90)*(math.pi/180))
      self.setNodeID(self.goal)
      #self.heap_.add(self.goal)
      #self.exploredset.add(self.start)
      
      print('Goal : ', self.goal)
      print('\n')
      
      #self.move_to_goal()
      
  
        
  def newcoords(self, length, angle):
      
      self.newx = length*math.sin(int(angle)*(math.pi/180))
      self.newy = length*math.cos(int(angle)*(math.pi/180))
      self.new_coord = [ self.newx, self.newy]
      return self.new_coord
     

      
  def getNeighbors(self):
      self.robotcontrol.get_laser_regions()
      #create temp node
      
      #for each lidar point create a node & define its coordinates
      for index, item in enumerate(self.robotcontrol.regions.items()):
           self.temp_del_ = self.newcoords(item[1], item[0])
           self.temp_ = Node()
           self.temp_.lidarscore = item[1]
           self.temp_.x = self.start.x + self.temp_del_[0]
           self.temp_.y = self.start.y + self.temp_del_[1]
           self.setNodeID(self.temp_)
           print(item)
           #self.costcalculator(self.temp_)
           
           
           if index == 0: 
              self.temp_.visionscore = (self.robotcontrol.regions["0"] 
              + self.robotcontrol.regions["45"])/2
             # self.new_direction = 0

           elif index == 1: 
              self.temp_.visionscore = (self.robotcontrol.regions["180"] 
              +self.robotcontrol.regions["135"])/2
             # self.new_direction = 180

           elif index == 2: 
              self.temp_.visionscore = (self.robotcontrol.regions["45"] 
              + self.robotcontrol.regions["90"] + self.robotcontrol.regions["0"])/3
              #self.new_direction = 45              
           elif index == 3: 
              self.temp_.visionscore = (self.robotcontrol.regions["135"] 
              + self.robotcontrol.regions["90"] + self.robotcontrol.regions["180"])/3
              #self.new_direction = 135

           elif index == 3: 
              self.temp_.visionscore = (self.robotcontrol.regions["90"] 
              + self.robotcontrol.regions["135"] + self.robotcontrol.regions["45"])/3           
              #self.new_direction = 90

           if self.temp_.lidarscore == 10 and self.temp_.visited != 1: 
              self.contain_ = self.potentials.contains(self.temp_)
              
              if self.contain_ == 1:
                 self.temp_.visited = 2
                 self.potentials.add(self.temp_)
            
           
           #self.heap_.add(self.temp_)
           
                            
     # print('Current Heap: ' , self.heap_.Heap )
      print('\n')
      #print('Potential Heap: ' , self.potentials.Heap )

  def goal_(self):
      ##Set start node info 
      self.start.setcoord(self.robotcontrol.getcurrentposition())
      self.start.visited = 1 
      self.start.theta = self.robotcontrol.getcurrentyaw()
      self.setNodeID(self.start)

      #set goal info 
      self.goal = self.heap_.pop()
      self.goal.theta = self.start.theta + ((int(self.new_direction)-90)*(math.pi/180))
      self.new_distance = self.robotcontrol.regions[str(self.new_direction)] 

      if self.new_distance < 7: 
        print("Searching for new goal")
        print('\n')
        self.robotcontrol.rotodom(-35)
        self.getNeighbors()
        self.goal_()

      self.heap_.clear(15)

               
  def move_to_goal(self):
      
      ### Fix heading 
      self.robotcontrol.rotodom(self.new_direction)
      
      ###Move to location
      self.robotcontrol.move_PID(self.start, self.goal)

      print(self.robotcontrol.getcurrentposition())
      print('\n')
      self.goal.setcoord(self.robotcontrol.getcurrentposition())
      
  
  def planner(self):
      clock_start = rospy.Time.now()
      timeout = rospy.Duration(120)
      print("Starting: Vision path planner")
      print('\n' )
      print( 'Time constraint: ' ,timeout)
      stop_ = False
      
      while (stop_ != True):#rospy.Time.now() - clock_start < timeout):
          self.setgoal()
          #self.getNeighbors()
          #self.goal_() 
          self.move_to_goal()
          #self.exploredset.add(self.heap_.pop())

          
      self.robotcontrol.stop_robot()
      print("Robot shutting Down...")
      self.robotcontrol.shutdownhook()
      

    
