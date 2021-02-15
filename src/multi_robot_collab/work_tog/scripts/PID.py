#!/usr/bin/env python

import math

from Node import Node

class PID(object):
  
  def __init__(self):
    #Initialize Gain Parameters
    self.ki = 0.1
    self.kp = 0.1
    self.kd = 0.1
    
    #Initialize Tolerance
    self.tolerance = 0.1
    
    #Initialize Error 
    self.error = 0
    
    #Initialize Previous Position for Derivative
    self.lastposition = 0
        
    #Initialize Integral Memory
    self.i_memory = [0]*50
    self.i_counter = 0

    
    #Initialize controller outputs 
    self.integral_out = 0
    self.proportional_out = 0
    self.derivative_out = 0
    
    #Initialize final output 
    self.velocity_out = 0 
    
    #Initialize End Criteria
    self.done = False

    #Initialize Current & Goal Nodes
    self.cp = Node()
    self.gp = Node()

    
  def Proportional(self):

    self.proportional_out = self.kp * self.error
    #print("Running Proportional") 
    #print("Proportional out: " , self.proportional_out) 
  
  def Integral(self):
    #print("Running Integral")
    self.i_memory[self.i_counter] = self.error
    self.i_counter += 1
    total = self.i_memory[0] * 2
    diff = 0
    #print("Integral out: " , self.integral_out)
    for index in range(len(self.i_memory)):
      diff = total - self.i_memory[index] 
      
    self.integral_out = self.ki * (total - diff)
  
  def Derivative(self):
    #print("Running Derivative")
    roc = (self.error - self.lastposition)/2
    
    self.derivative_out = roc * self.kd
   # print("Derivative out: " , self.derivative_out)
    self.lastposition = self.error
  
  def Feedback(self):
    #print("Running Feedback")
    self.error = math.sqrt(pow(abs(self.cp.x - self.gp.x),2) 
                  + pow(abs(self.cp.y - self.gp.y),2))
    #print("Error : ", self.error)
  
  def Combine(self):
    #print("Running Combine")
    self.velocity_out = self.integral_out + self.proportional_out + self.derivative_out
    #if self.velocity_out > 0.6:
    #     self.velocity_out = 0.6
    self.velocity_out = 0.6
   
  
  
  def reset(self):
    #Initialize Error 
    self.error = 0
    
    #Initialize Previous Position for Derivative
    self.lastposition = 0
        
    #Initialize Integral Memory
    self.i_memory = [0]*50
    self.i_counter = 0
    
    #Initialize controller outputs 
    self.integral_out = 0
    self.proportional_out = 0
    self.derivative_out = 0
    
    #Initialize final output 
    self.velocity_out = 0 
    
    #Initialize End Criteria
    self.done = False

  def Done(self): 
    #print("Running Done")
    x_comp = self.cp.x + (self.cp.x * self.tolerance)
    y_comp = self.cp.y + (self.cp.y * self.tolerance)

    if self.velocity_out <= 0.15: #
      self.done = True 
    if x_comp == self.gp.x and y_comp == self.gp.y:
      self.done = True 
      
    return self.done   
  
  
  def one_shot(self, current_position, goal_position):
    self.cp = current_position
    self.gp = goal_position

    #Runs entire PID controller  
    self.Feedback()
    self.Proportional()
    self.Integral()
    self.Derivative()
    self.Combine()
    self.Done()
  
    
