import numpy
import cv2

class HybridNode:
	def __init__(self, x, y, th, definition=40, ):
		self.x = x
		self.y = y
		self.th = th
		self.definition = definition

		self.parent = None
		self.children = []

	def addParent(self, par):
		self.parent = par 

	def getParent(self):
		return self.parent

	def addChild(self, child):
		self.children.append(child)

	def getChildren(self):
		return self.children

	def extend(self, speed, phi):
		midways = []
		jump = speed/self.definition
		curr_th = self.th
		x = self.x
		y = self.y
		for i in range(self.definition):
			x = jump*numpy.cos(curr_th) + x
			y = jump*numpy.sin(curr_th) + y
			curr_th = curr_th + phi
			midways.append((x, y, curr_th))
		return midways

	def getNeighbors(self, speed, phiMax, num):
		dt = phiMax*2/(num-1)
		curr_phi = -phiMax
		results = ([], [])
		for i in range(num):
			curr_path = self.extend(speed, curr_phi)
			tx, ty, tth = curr_path[-1]
			results[0].append(HybridNode(tx, ty, tth))
			results[1].append(curr_path)
			curr_phi += dt
		return results


if __name__ == "__main__":
	cv2.namedWindow("test", cv2.WINDOW_NORMAL)
	#fix orientation
	x, y, th = 250, 150, numpy.pi
	n = HybridNode(x, y, th)
	terrain = numpy.zeros((300, 300, 3))
	terrain[:,:] = (255, 255, 255)
	#cv2.imshow("test", terrain)
	#cv2.waitKey(0)
	colors = [(0, 0, 0), (0, 0, 255), (0, 255, 255), (0, 255, 0), (255, 255, 0), (255, 0, 0), (255, 0, 255)]
	
	cv2.circle(terrain, (x, y), 1, (0,255,0))
	nodes = [n]

	for i in range(343):
		curr_colr = colors[i%7]
		curr_node = nodes.pop(0)
		extension = curr_node.getNeighbors(40, 0.02, 7)
		nodes.extend(extension[0])

		for path in extension[1]:
			for dot in path:
				terrain[int(dot[0]), int(dot[1])] = curr_colr

	cv2.imshow("test", terrain)
	cv2.waitKey(0)

