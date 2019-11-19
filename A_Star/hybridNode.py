import numpy
import cv2

class HybridNode:
	def __init__(self, x, y, th, cost, definition=200):
		self.x = x
		self.y = y
		self.th = th
		self.definition = definition

		self.parent = None
		self.children = []
		self.path = None
		self.cost = 0
		self.backwards = True

	def __str__(self):
		return "{0}x, {1}y, {2}th, {3}c".format(self.x, self.y, self.th, self.cost)

	def addParent(self, par):
		self.parent = par 

	def getParent(self):
		return self.parent

	def addChild(self, child):
		self.children.append(child)

	def getChildren(self):
		return self.children

	def extendPath(self, speed, phi):
		midways = []
		jump = speed/self.definition
		dth = speed*numpy.tan(phi)
		curr_th = self.th
		x = self.x
		y = self.y
		for i in range(self.definition):
			x = jump*numpy.cos(curr_th) + x
			y = jump*numpy.sin(curr_th) + y
			curr_th = curr_th + dth
			midways.append((x, y, curr_th))

		newNode = HybridNode(x, y, curr_th, self.cost+speed)
		newNode.path = midways

		return newNode

	def getNeighbors(self, speed, maxRadius, num):
		phiMax = numpy.arcsin(1/(maxRadius*25))
		dt = phiMax*2/(num-1)
		curr_phi = -phiMax
		self.children = []

		#forwards propogation
		for i in range(num):
			self.children.append(self.extendPath(speed, curr_phi))
			curr_phi += dt

		#allow backwards propogation
		if self.backwards:
			curr_phi = -phiMax
			for i in range(num):
				self.children.append(self.extendPath(-speed, curr_phi))
				curr_phi += dt

		return self.children

	def distTo(self, other):
		dx = self.x - other.x
		dy = self.y - other.y
		dt = self.th - other.th
		return numpy.sqrt(dt**2 + dx**2 + dy**2)


if __name__ == "__main__":
	cv2.namedWindow("test", cv2.WINDOW_NORMAL)
	#fix orientation
	x, y, th = 500, 500, numpy.pi*2/4
	n = HybridNode(x, y, th, 0)
	terrain = numpy.zeros((1000, 1000, 3))
	terrain[:,:] = (255, 255, 255)
	#cv2.imshow("test", terrain)
	#cv2.waitKey(0)
	colors = [(0, 0, 0), (0, 0, 255), (0, 255, 255), (0, 255, 0), (255, 255, 0), (255, 0, 0), (255, 0, 255)]
	
	nodes = [n]
	neighbors = 3

	for i in range(1 + 2*neighbors + (2*neighbors)**2):
		curr_colr = colors[i%7]
		curr_node = nodes.pop(0)
		extension = curr_node.getNeighbors(100, 600, neighbors)
		nodes.extend(extension)

		for node in extension:
			for dot in node.path:
				x, y = int(dot[0]), 1000-int(dot[1])
				if x>=1000 or x<0 or y>=1000 or y<0:
					continue
				terrain[1000-int(dot[1]), int(dot[0])] = curr_colr

	cv2.circle(terrain, (x, y), 1, (0,255,0))

	cv2.imshow("test", terrain)
	cv2.waitKey(0)

'''
1600 -> 422 484   62
800  -> 453 484   31
400  -> 467 484   17
'''

