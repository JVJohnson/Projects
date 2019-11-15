#Imports
import numpy
import cv2
import queue
import math
#import matplotlib.pyplot as plt
import time
import csv
import cProfile, pstats, io

'''Pseudocode

	Establish data structures
		-Map: 2d array of integer spacing (numpy array)
			-has freespace and/or occupiedSpace indicators(Pixel color)
		-Frontier: priority queue
		-Node: Int x, Int y, Double theta, Double realX, Double realY
			   Node Parent

	Iterate While:
		-Frontier is NOT empty and
		-Goal is NOT reached

		Pick node from priority queue (sorted by heuristic)
		check it is a valid node
			-is it within bounds?
			-is it in freespace?
		Call Function: Find neighbors of current node
		for each of the neighbors:
			apply heuristic to neighbor
			put neighbor in frontier queue according to heuristic 




Function: Find Neighbors
	Params: Parent Node
	Optional Params: number of neighbors, max Turning angle, speed/distance

	Utilizes the Max turning angle of our robot to spawn a certain number of
	neighbor nodes

	Take Parent realX, realY, and Theta
	Apply kinematic equations for (numOfNeighbors) turning angles
		- start at max negative steering angle
		- end at max positive steering angle
		- Good number is probably about 5-7 neighbors
		Create Nodes At said positions (store realX, realY and Theta)


'''	
class HybridAStar:
	def __init__(self, terrain, start, end):
		self.terrain = terrain
		self.start = start
		self.end = end
		self.speed = 100
		self.radius = 10
		self.definition = 10
		self.num_neighbors = 7


if __name__ == "__main__":
	dim = 1000
	terrain = numpy.full((dim, dim, 3), 255)
	start = (500, 800, 0)
	end = (500, 200, numpy.pi/2)

	HybridAStar(terrain, start, end)