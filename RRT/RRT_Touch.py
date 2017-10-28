import cv2
import numpy as np
import math
import random


#Constants
TITLE = "RRT"

COLOR_BG = (255,255,255)
COLOR_EDGE = (0,0,0)
COLOR_NODE = (0,0,0)
COLOR_OBS = (255,0,0)


MAP_WIDTH = 5000
MAP_HEIGHT = 5000
START = [   
            
            
            (   int(MAP_WIDTH-2   )   ,int(MAP_HEIGHT-1  )   )

            ]  
                



OBS =   [
            (   (int(MAP_WIDTH/2),int(MAP_HEIGHT/3)),(int(MAP_HEIGHT*7/24),int(MAP_HEIGHT*5/6))     ),
            (   (int(MAP_HEIGHT*7/24),int(MAP_HEIGHT*5/6)),(int(MAP_HEIGHT*17/24),int(MAP_HEIGHT*5/6))     ),
            (   (int(MAP_HEIGHT*17/24),int(MAP_HEIGHT*5/6)),(int(MAP_WIDTH/2),int(MAP_HEIGHT/3))     ),
        ]

CIRCS = [
            (int(MAP_WIDTH/2),int(MAP_HEIGHT/3), 950)
        ]
CIRC_RES = 100



MAX_NODES = 1000
MOD = MAX_NODES / 200


LINE_THICKNESS = 9#int( math.ceil((MAP_WIDTH + MAP_HEIGHT)/500)   )

NODE_RADIUS =  LINE_THICKNESS +1

EPSILON= int( (MAP_WIDTH + MAP_HEIGHT)/2     * .15)
MU =int( (MAP_WIDTH + MAP_HEIGHT)/2     * .01)

def Draw(im, k, OBS, redraw):
    '''
    Draws the map

    :param im: the image to draw
    :param k: the list of nodes
    :param obs: obstacles to draw

    
    '''
    for segment in OBS:
        cv2.line(im, segment[0], segment[1], COLOR_OBS, thickness=LINE_THICKNESS+2)
    for node in redraw:
        cv2.line(im, node, redraw[node], COLOR_BG, thickness=LINE_THICKNESS)
    
    for node in k:  #draws all the nodes (not just new ones in case there is an update feature later)
        
        #cv2.circle(im, node, NODE_RADIUS, COLOR_NODE)
        cv2.line(im, node, k[node], COLOR_EDGE, thickness=LINE_THICKNESS)
    
    
    cv2.imshow(TITLE, im)
    cv2.waitKey(10)


    

def EndIm(im, k):
    '''
    called at the end to keep image up

    '''
    
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    



def distance(a,b):
    '''
    Calculates euclidean distance between two points

    :param a: point 1
    :param b: point 2

    :return dist: the euclidean distance (a^2 + b^2 = c^2)
    '''
    return math.sqrt(               \
                (a[0] - b[0]) **2 + \
                (a[1] - b[1]) **2 )





def distanceSortLambda(origin):
    '''
    returns a lambda to sort a function by distance from a point
    
    :param origin: the point to find distance from

    :return lambda: function that finds distance between origin and x
    '''
    return lambda x: distance(origin, x)




def rewire(newNode, k, costs, im, OBS):

    '''
    rewires nearby nodes to a new node when generated

    :param newNode: the new node to be checked
    :param k: the map of nodes
    :param costs: the cost of all the nodes
    :param im: image being drawn on

    :return redraw: dict comtaining lines to redraw

    '''
    redraw = {}
    for node in k:
        if k[newNode] == node: continue
        if distance(node, newNode) > EPSILON: continue
        if collides(node, newNode, OBS): continue
        
        newCost = costs[newNode] + distance(node, newNode)
        if newCost < costs[node]:
            redraw[node]= k[node]
        
            k[node] = newNode
            costs[node] = newCost
    return redraw




def ccw(A,B,C):
    '''
    Determines if the points are counter clock-wise
        got this from stackoverflow.com/questions/3838329/how-can-i-check-if-two-line-segments-intersect

    :param A: x,y point
    :param B: another x,y point
    :param C: one more x,y point

    :return: boolean is counter clockwise or not
    '''
    First = (C[1] - A[1]) * (B[0] - A[0])
    Second =(B[1] - A[1]) * (C[0] - A[0])

    return First > Second




def intersect(A,B,C,D):
    '''
    detects if two line segments intersect

    :param A,B,C,D: x,y points of line segment AB and CD

    :return: boolean if the lines intersect
    '''
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


    
def collides(node, newNode, Obs):
    '''
    checks if two points intersect with the obstacles

    :param node, newNode: nodes to check a segment bewteen
    :param Obs: list of line segments to check

    :return: boolean if it collides
    '''
    collides = False
    for segment in Obs:
        if intersect(node, newNode, segment[0], segment[1]):
            collides = True
            break
    return collides



def makeCircle(midpoint, rad, res):
    '''
    Makes a circle into multiple line segments

    :param midpoint: center of the circle
    :param rad: radius of the circle
    :param res: number of segments to simulate the circle

    :return circle: dict of points outlining the circle
    '''
    angleInc = math.pi * 2 / res

    circle = []

    prev = (midpoint[0] + rad, midpoint[1])
    for i in range(1,res+1):
        ang = i * angleInc
        x = int(rad * math.cos(ang)) + midpoint[0]
        y = int(rad * math.sin(ang)) + midpoint[1]
        circle.append((prev, (x,y)))
        prev = (x,y)
        
    return circle




def main():
    #initializing all my stuff


    cv2.namedWindow(TITLE, flags=cv2.WINDOW_NORMAL)

    
    Map = np.zeros((MAP_HEIGHT, MAP_WIDTH, 3))
    Map[:,:] = COLOR_BG

    for circle in CIRCS:
        center = (circle[0], circle[1])
        rad = circle[2]
        OBS.extend( makeCircle(center, rad, CIRC_RES))
        
        cv2.circle(Map, center, rad, COLOR_OBS)
    
    


    
    
    k       = {}                #dict of nodes
    costs   = {}                #dict of costs
    redraw  = {}                #dict of nodes to rewire
    for point in START:
        k[point]        = point
        costs[point]    = 0

    print("Initialized")
    cv2.waitKey(0)

    

                        #find set number of nodes
    while len(k) < MAX_NODES:
        newNodeY = random.randint(1, MAP_HEIGHT) -1
        newNodeX = random.randint(1, MAP_WIDTH) - 1
        newNode = (newNodeX, newNodeY)

       
        #sort nodes by distance to newNode
        nodes = sorted( k, key=distanceSortLambda(newNode)   )
        parent = nodes[0]
        minCost = costs[parent] + distance(newNode, parent)

        #iterate through closest nodes to find lowest cost
        for node in nodes[0:4]:
            compCost = costs[node] + distance(newNode, node)
            if compCost < minCost:
                minCost = compCost
                parent = node
            



        #EpsCheck = 2*EPSILON / len(k)        #useful for an odd pattern

        #check to see if new node is just too far
        newDist = distance(newNode, parent)
##        print(newNode)

        #reduces distance to epsilon if over
        if newDist > EPSILON:
            continue
            '''
            #curr distance
            xDist = -(parent[0] - newNode[0])
            yDist = -(parent[1] - newNode[1])

            #mult by epsilon
            xDist *= EPSILON
            yDist *= EPSILON

            newNodeX = int(xDist/newDist) + parent[0]
            newNodeY = int(yDist/newDist) + parent[1]

            #divide by curr distance effectively D*Epsilon/D, so EPSILON 
            newNode = (newNodeX, newNodeY)
            '''

        
        #collides with obstacle
        if collides(parent, newNode, OBS):
            continue
        #out of bounds
        if(newNode[0] > MAP_WIDTH  or newNode[0] < 0):
            continue
        if(newNode[1] > MAP_HEIGHT or newNode[1] < 0):
            continue
    
        #add newNode into lists
        costs[newNode] = minCost
        k[newNode] = parent
        redraw  =  {**redraw, **rewire(newNode, k, costs, Map, OBS)   }
        
        #update image
        #Draw(Map, k, OBS, redraw)
        if(len(k) % MOD ==0):
            print("" + str(len(k)) + " / " + str(MAX_NODES) + "  (" + str(100.0*len(k)/MAX_NODES) + "%)")
            Draw(Map, k, OBS, redraw)
            redraw = {}
        #if(len(k) % 50 ==0):
         #   cv2.imshow(TITLE, Map)
          #  cv2.waitKey(1)

        
    
    
    Draw(Map, k, OBS, redraw)
    EndIm(Map, k)





if __name__ == "__main__":
    main()
