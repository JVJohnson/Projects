#Imports
import numpy
import cv2
import queue
import math
#import matplotlib.pyplot as plt
import time
import csv
import cProfile, pstats, io
from hybridNode import HybridNode



class HybridAStar:
	def __init__(self, terrain, start, end):
		self.terrain = terrain
		self.start = start
		self.end = end
		self.speed = 10
		self.radius = 20
		self.definition = 160
		self.num_neighbors = 7
		self.admittance = 7
		self.nodes = {}

		self.nodes[start] = self.nodeCost(self.start)

	def nodeCost(self, node):
		return node.cost + self.end.distTo(node)




if __name__ == "__main__":
	dim = 1000
	terrain = numpy.full((dim, dim, 3), 255)
	start = HybridNode(500, 800, 0, 0)
	end = HybridNode(500, 200, numpy.pi/2, 0)
	
	plan = HybridAStar(terrain, start, end)
