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


MAP_WIDTH = 6000
MAP_HEIGHT = 4000
START = [   
            
            (   int(   0        )   ,   int(0           )   ),
            (   int(MAP_WIDTH/2 )   ,   int(0           )   ),
            (   int(MAP_WIDTH   )   ,   int(0           )   ),

            
            (   int(MAP_WIDTH/4 )   ,   int(MAP_HEIGHT/6)   ), 
            (3* int(MAP_WIDTH/4 )   ,   int(MAP_HEIGHT/6)   ),

            (   int(   0        )   ,   int(MAP_HEIGHT/3)   ),
            (   int(MAP_WIDTH/2 )   ,   int(MAP_HEIGHT/3)   ),
            (   int(MAP_WIDTH   )   ,   int(MAP_HEIGHT/3)   ),

            
            (   int(MAP_WIDTH/4 )   ,   int(MAP_HEIGHT/2)   ), 
            (3* int(MAP_WIDTH/4 )   ,   int(MAP_HEIGHT/2)   ),

            (   int(   0        )   , 2*int(MAP_HEIGHT/3)   ),
            (   int(MAP_WIDTH/2 )   , 2*int(MAP_HEIGHT/3)   ),
            (   int(MAP_WIDTH   )   , 2*int(MAP_HEIGHT/3)   ),

            
            (   int(MAP_WIDTH/4 )   , 5*int(MAP_HEIGHT/6)   ), 
            (3* int(MAP_WIDTH/4 )   , 5*int(MAP_HEIGHT/6)   ),
            
            (   int(   0        )   ,   int(MAP_HEIGHT  )   ),
            (   int(MAP_WIDTH/2 )   ,   int(MAP_HEIGHT  )   ),
            (   int(MAP_WIDTH   )   ,   int(MAP_HEIGHT  )   )

            ]  
                



OBS =   [
        ((400,600), (400,400), (600,400), (600,600))
        ]



MAX_NODES = 10000
MOD = MAX_NODES / 100


LINE_THICKNESS = int( math.ceil((MAP_WIDTH + MAP_HEIGHT)/2000)   )

NODE_RADIUS =  LINE_THICKNESS +1

EPSILON= int( (MAP_WIDTH + MAP_HEIGHT)/2     * .05)


def Draw(im, k, obs, redraw):
    '''
    Draws the map

    :param im: the image to draw
    :param k: the list of nodes
    :param obs: obstacles to draw

    
    '''
    for node in redraw:
        cv2.line(im, node, redraw[node], COLOR_BG, thickness=LINE_THICKNESS)
    for node in k:  #draws all the nodes (not just new ones in case there is an update feature later)
        
        cv2.circle(im, node, NODE_RADIUS, COLOR_NODE)
        cv2.line(im, node, k[node], COLOR_EDGE, thickness=LINE_THICKNESS)
    '''for obstacle in obs:
        cv2.rectangle(im, obstacle[0], obstacle[1], COLOR_OBS, thickness = EPSILON) 
    '''
    
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

def rewire(newNode, k, costs, im):

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
        
        newCost = costs[newNode] + distance(node, newNode)
        if newCost < costs[node]:
            redraw[node]= k[node]
        
            k[node] = newNode
            costs[node] = newCost
    return redraw        


    

def main():
        #initializing all my stuff


    cv2.namedWindow(TITLE, flags=cv2.WINDOW_NORMAL)

    
    Map = np.zeros((MAP_HEIGHT, MAP_WIDTH, 3))
    Map[:,:] = COLOR_BG

    Obs = []
    k       = {}                #dict of nodes
    costs   = {}                #dict of costs
    for point in START:
        k[point]        = point
        costs[point]    = 0

    print("Initialized")


    

                        #find set number of nodes
    while len(k) < MAX_NODES:
        newNodeY = random.randint(1, MAP_HEIGHT) -1
        newNodeX = random.randint(1, MAP_WIDTH) - 1
        newNode = (newNodeX, newNodeY)

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
            

        #EpsCheck = 2*EPSILON / len(k)        #useful for an odd pattern

        #check to see if new node is just too far
        if distance(newNode, parent) > EPSILON:
            
            continue

        #add newNode into lists
        costs[newNode] = minCost
        k[newNode] = parent

        redraw = rewire(newNode, k, costs, Map)

        
        #update image
        #Draw(Map, k, Obs, redraw)
        #if(len(k) % MOD ==0):
         #   print("" + str(len(k)) + " / " + str(MAX_NODES) + "  (" + str(100.0*len(k)/MAX_NODES) + "%)")
          #  Draw(Map, k, Obs, redraw)
        #if(len(k) % 50 ==0):
         #   cv2.imshow(TITLE, Map)
          #  cv2.waitKey(1)

        
    
    
    Draw(Map, k, Obs, redraw)
    EndIm(Map, k)





if __name__ == "__main__":
    main()
