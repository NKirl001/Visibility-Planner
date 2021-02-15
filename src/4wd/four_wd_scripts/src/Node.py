#!/usr/bin/env python


class Node(object):
    def __init__(self):
        #Node Coordinates
        self.x = 0
        self.y = 0
        self.theta = 0

            
        #Vision score
        self.visionscore = 0
        self.lidarscore = float("inf")

        ##For revisiting a node        
        #Node Costs
        #gcost = cost from node to goal
        self.gCost = 0
        #hcost = cost for step
        self.hCost = 0
        #total cost
        self.fCost = float("inf")

        #Node Id
        self.NodeID = float("inf")
        #Parent
        self.parent = 0
          
        #Explored
        self.visited = 0
       

    def Costs(g, h):
        self.gCost = g
        self.hCost = h

    def getFcost():
        self.fCost = g + h
        return self.fCost

    def setcoord(self, coord):#x_, y_):
        self.x = coord[0]
        self.y = coord[1]

    def setNodeID(self, num):
        self.NodeID = num

    def getNodeID(self):
        return self.NodeID

    def __repr__(self):
        return repr(('x: {} ||  y:{}  || theta: {} Vision Score: {} || Lidar Score:',self.x, self.y, self.theta, self.visionscore ,self.lidarscore))
