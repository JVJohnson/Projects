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


MAP_WIDTH = 1000
MAP_HEIGHT = 1000
#START = (   int(MAP_WIDTH/2)  , int(MAP_HEIGHT/2)  )
START = (50,50)

GOAL = (MAP_WIDTH-50, MAP_HEIGHT - 50)

OBS =   [
        ((400,600), (400,400), (600,400), (600,600))
        ]



MAX_NODES = 5000

LINE_THICKNESS = int( math.ceil((MAP_WIDTH + MAP_HEIGHT)/1500)   )

NODE_RADIUS =  LINE_THICKNESS + 0

EPSILON= int( (MAP_WIDTH + MAP_HEIGHT)/2     * .13)



def Draw(im, k, obs):
    '''
    Draws the map

    :param im: the image to draw
    :param k: the list of nodes
    :param obs: obstacles to draw

    
    '''
    

    for node in k:  #draws all the nodes (not just new ones in case there is an update feature later)
        
        cv2.circle(im, node, NODE_RADIUS, COLOR_NODE)
        cv2.line(im, node, k[node], COLOR_EDGE, thickness=LINE_THICKNESS)
    '''for obstacle in obs:
        cv2.rectangle(im, obstacle[0], obstacle[1], COLOR_OBS, thickness = EPSILON) 
    '''
    if GOAL in k:

        currNode = GOAL
        while(currNode is not START):
            cv2.line(im, currNode, k[currNode], (255,0,255), thickness=LINE_THICKNESS)
            currNode = k[currNode]

    cv2.circle(im, START,  4, (255,0,255), thickness=3)
    cv2.circle(im, GOAL,   4, (255,0,255), thickness=3)

    
    cv2.imshow(TITLE, im)
    cv2.waitKey(1)

    

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

def rewire(newNode, k, costs, im):

    '''
    rewires nearby nodes to a new node when generated

    :param newNode: the new node to be checked
    :param k: the map of nodes
    :param costs: the cost of all the nodes
    :param im: image being drawn on

    '''
    for node in k:
        if k[newNode] == node: continue
        if distance(node, newNode) > EPSILON: continue
        
        newCost = costs[newNode] + distance(node, newNode)
        if newCost < costs[node]:
            cv2.line(im, node, k[node], (255,255,255), thickness=LINE_THICKNESS)
            k[node] = newNode
            costs[node] = newCost
        


    

def main():
        #initializing all my stuff


    prev_cost = 0
    cv2.namedWindow(TITLE, flags=cv2.WINDOW_NORMAL)

    
    Map = np.zeros((MAP_HEIGHT, MAP_WIDTH, 3))
    Map[:,:] = COLOR_BG

    Obs = []
    k = {START: START}  #dict of nodes
    costs = {START:0}

    

    Draw(Map, k, Obs)

                        #find set number of nodes
    while len(k) < MAX_NODES:
        newNodeY = random.randint(1, MAP_HEIGHT) -1
        newNodeX = random.randint(1, MAP_WIDTH) - 1
        newNode = (newNodeY, newNodeX)

        '''if Map[newNodeY, newNodeX, 0] != COLOR_BG[0] and \
           Map[newNodeY, newNodeX, 1] != COLOR_BG[1] and \
           Map[newNodeY, newNodeX, 2] != COLOR_BG[2] :
            continue
        '''
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
            

        #EpsCheck = EPSILON / len(k)        #useful for an odd pattern

        #check to see if new node is just too far
        if distance(newNode, parent) > EPSILON:
            
            continue

        #add newNode into lists
        costs[newNode] = minCost
        k[newNode] = parent

        rewire(newNode, k, costs, Map)

        if(distance(newNode,GOAL) < EPSILON):
            minCost = costs[newNode] + distance(newNode, GOAL)
            if prev_cost ==0 or minCost < prev_cost:
                prev_cost = minCost
                k[GOAL] = newNode
                costs[GOAL] = costs[newNode] + distance(newNode, GOAL)
            
        #update image
        Draw(Map, k, Obs)

        
    
    
    #Draw(Map, k, Obs)
    EndIm(Map, k)





if __name__ == "__main__":
    main()
