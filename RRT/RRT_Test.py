import cv2
import numpy as np
import math
import random


#Constants
TITLE = "RRT"

COLOR_BG = (255,255,255)
COLOR_EDGE = (0,0,0)
COLOR_NODE = (0,0,0)

MAP_WIDTH = 1000
MAP_HEIGHT = 1000
START = (   int(MAP_HEIGHT/2)  , int(MAP_WIDTH/2)  )

MAX_NODES = 1000

LINE_THICKNESS = int(   (MAP_WIDTH + MAP_HEIGHT)/1000   )

NODE_RADIUS = LINE_THICKNESS + 1

EPSILON= (MAP_WIDTH + MAP_HEIGHT)/2     * .1


def Draw(im, k):
    '''
    Draws the map

    :param im: the image to draw
    :param k: the list of nodes
    '''
    for node in k:
        
        cv2.circle(im, node, NODE_RADIUS, COLOR_NODE)
        cv2.line(im, node, k[node], COLOR_EDGE, thickness=LINE_THICKNESS)
    
    cv2.imshow(TITLE, im)
    cv2.waitKey(1)

    

def EndIm():
    '''
    called at the 

    '''
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def distance(a,b):
    return math.sqrt(               \
                (a[0] - b[0]) **2 + \
                (a[1] - b[1]) **2 )



def distanceSortLambda(origin):
    pass
    
def calcCost(a,b,costs):
    pass #return lambda

 


def main():
        #initializing all my stuff


    cv2.namedWindow(TITLE, flags=cv2.WINDOW_NORMAL)


    
    Map = np.zeros((MAP_HEIGHT, MAP_WIDTH, 3))
    Map[:,:] = COLOR_BG

    k = {START: START}
    costs = {START:0}

    Draw(Map, k)
    
    while len(k) < MAX_NODES:
        newNodeX = random.randint(0, MAP_WIDTH)
        newNodeY = random.randint(0, MAP_HEIGHT)
        newNode = (newNodeY, newNodeX)

        dist = distance(newNode, START)
        parent = START

        for node in k:
            compDist = distance(newNode, node)
            if compDist < dist:
                dist = compDist
                parent = node

        if dist > EPSILON:
            
            continue
        

        
        k[newNode] = parent
        
        Draw(Map, k)
    
    
    
    EndIm()





if __name__ == "__main__":
    main()
