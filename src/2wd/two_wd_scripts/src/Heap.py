#!/usr/bin/env python
from Node import Node
import operator

class Heap(object):
    def __init__(self, size_):
        self.current_count = 0
        self.Heap = []
        self.HeapSize(size_)

    def HeapSize(self, size):
        self.maxSize = size
        self.Heap = [Node()] * self.maxSize

    def add(self, node):
        self.item = node
        if self.current_count +1 > self.maxSize:
            print('Unable to add to heap: Exceeding Heap Size')

        #Assign nodeID based on heap index
        #self.item.setNodeID(self.current_count)
        #print(self.item.NodeID )
        #print(self.current_count)
        #Add to array
        self.Heap[self.current_count] = self.item

        #Sort Array
        keyfun= operator.attrgetter("visionscore")
        #self.Heap.sort(key=lambda x: self.item.fCost, reverse=True)
        self.Heap.sort(key=keyfun, reverse=True)

        #Incrememnt
        self.current_count += 1

    def pop(self):
        #get first item from heap
        self.first = self.Heap[0]

        #decrease current item current_count
        self.current_count -= 1

        #resort Array
        #add last node to front of list
        self.Heap[0] = self.Heap[self.current_count]
        #remove duplicate node
        #self.Heap[self.current_count] = None
        self.item = self.Heap[0]
        self.item.setNodeID(0)

        #Sort Array
        keyfun= operator.attrgetter("visionscore")
        #self.Heap.sort(key=lambda x: self.item.fCost, reverse=True)
        self.Heap.sort(key=keyfun, reverse=True)

        return self.first

                
    def clear(self, size_):
        self.current_count = 0
        self.Heap = []
        self.HeapSize(size_)



    def contains(self, node):
        self.temp_ = node
        
        for item in self.Heap:
            if self.temp_.NodeID == item.NodeID:
               return 1         
        
        #return print('True' if self.Heap[self.item.NodeID] == self.item else
        #       'False')
