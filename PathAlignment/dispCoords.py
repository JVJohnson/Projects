import cv2
import csv
import numpy

height = 200
width = 200


#Mess around with these to change position of the slam path
#BEST based on beginning: 1.25, 1.00, (101,99), True

scaleSlam = 1.37    #scale of path  
rotateSlam= 0.98    #angle of path(around center, defined below
centerSlam= (101, 99) #'center' of path (start)
flipSlam  = True   #whether the path should be flipped 
def readCsv(filename, cent= (width/2, height/2), rotate=0, scale=1, flip=False):
    '''reads in csv of points (time, x, y, z) and returns x,y 
    :param filename: csv file name

    :return list: list of (x,y) points
    '''
    points = []
    with open(filename, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for row in reader:
            points.append(   ( float(row[1]) , float(row[2]) )    )
    return points

def transformPoints(points, cent= (width/2, height/2), rotate=0, scale=1, flip=False):
    '''transforms points using defined parameters
    :param (x,y) list of points
    :param cent: center for points to be translated to
    :param rotate: radians to rotate points about center
    :param scale: scalar to stretch points by from center
    :param flip: boolean wether to flip points in x direction

    :return list: (x,y) of points with applied geometry
    '''
    points = numpy.array(points)
    rotMat = numpy.array( [[numpy.cos(rotate), numpy.sin(rotate)], [-numpy.sin(rotate), numpy.cos(rotate)]] )
    pointsPrime = numpy.matmul(points,rotMat)
    pointsPrime*=scale
    if flip:
        pointsPrime = numpy.matmul(pointsPrime, numpy.array([[-1,0],[0,1]])  )
    pointsPrime = numpy.add(pointsPrime, cent)
    return [tuple([int(z) for z in x]) for x in pointsPrime]



def plotPoints(points, image, color=(0,255,0)):
    '''plots points on an image
    :param points: list of (x,y) points
    
    '''
    point=0
    prevPoint = points[0]
    for cent in points:
        
        #cv2.circle(im, cent, 1, color)
        cv2.line(im, cent, prevPoint, color, 1)
        prevPoint = cent
        #print((point, cent))
        #cv2.imshow("points", im)
        #cv2.waitKey(10)
        point+=1
    return





if __name__ == "__main__":
    cv2.namedWindow("points", cv2.WINDOW_NORMAL)
    im = numpy.zeros((height, width, 3), numpy.uint8)
    im[:,:] = (255,255,255)


    fname = "gt-husky-indoor-cutoff.csv"
    fname2 = "msckf_poses.csv"
    pts = transformPoints(readCsv(fname))
    plotPoints(pts, im)
    
    pts2 = readCsv(fname2)
    pts2Prime = transformPoints(pts2, centerSlam, rotateSlam, scaleSlam, flipSlam)
    plotPoints(pts2Prime, im, (255,0,0))
    cv2.imshow("points", im)
    cv2.waitKey(0)
