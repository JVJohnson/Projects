import cv2
import csv
import numpy
import math
from scipy.optimize import minimize
from scipy.spatial.distance import cdist
resScalar = 10
resThick = resScalar/4
height = 200 * resScalar
width = 200* resScalar


#Mess around with these to change position of the slam path
#BEST based on beginning: 1.25, 1.00, (101,99), True

scaleSlam = 1.00       #scale of path  
rotateSlam= 00         #angle of path(around center, defined below
centerSlam= [100, 100] #'center' of path (start)
flipSlam  = False      #whether the path should be flipped 
squishSlam= [1, 1] #percentage multiplication of x and y separately
def readCsv(filename):
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

def transformPoints(points, cent= (width/2, height/2), rotate=0, scale=1, flip=False, squish=(1,1)):
    '''transforms points using defined parameters
    :param (x,y) list of points
    :param cent: center for points to be translated to
    :param rotate: degrees to rotate points about center
    :param scale: scalar to stretch points by from center
    :param flip: boolean wether to flip points in x direction
    :param squish: tuple multiple to stretch (x, y) from center

    :return list: (x,y) of points with applied geometry
    '''
    points = numpy.array(points)
    rotate*=math.pi/180
    rotMat = numpy.array( [[numpy.cos(rotate), numpy.sin(rotate)], [-numpy.sin(rotate), numpy.cos(rotate)]] )
    pointsPrime = numpy.matmul(points,rotMat)
    pointsPrime*=scale* resScalar
    squishArray = numpy.absolute( numpy.array(squish) ) #use absolute so it doesn't flip
    pointsPrime = pointsPrime*squishArray
    if flip:
        pointsPrime = numpy.matmul(pointsPrime, numpy.array([[-1,0],[0,1]])  )
    pointsPrime = numpy.add(pointsPrime, cent)
    return [tuple([int(z) for z in x]) for x in pointsPrime]



def plotPoints(points, image, color=(0,255,0)):
    '''plots points on an image
    :param points: list of (x,y) points
    
    '''
    prevPoint = points[0]
    for cent in points:
        #print(cent)
        #cv2.circle(im, cent, 1, color)
        cv2.line(im, cent, prevPoint, color, resThick)
        prevPoint = cent
        #cv2.imshow("points", im)
        #cv2.waitKey(1)

    return

def RMSE(mat1, mat2):
    '''calculates rmse between two lists
    :param mat1: first list, shortest
    :param mat2: second list

    :return float: rmse betweenthe two matricies
    '''
    m1, m2 = numpy.array(mat1), numpy.array(mat2)

    rmse = -1
    if m1.shape == m2.shape:        #same shape, speed up drastically
        rmse= numpy.sqrt(   ( (m1-m2                   )**2 ).mean()   ) /resScalar
    else:                           #different shape, slower
        rmse= numpy.sqrt(   ( (cdist(m2,m1).min(axis=0))**2 ).mean()   ) /resScalar
    return rmse



def optRMSE(x, flip, mat1, mat2):
    '''helper function for optimization rmse between the two lists
    :param x: list of params; center, rotate, scale, squish
    :param flip: boolean whether to flp the path
    :param mat1: original matrix
    :param mat2: matrix to transform onto mat1

    :return float: rmse between the original matrix (mat1) and transformed mat2
    '''
    mat2Prime = transformPoints(mat2, (x[0], x[1]), x[2], x[3], flip, (x[4], x[5]) )
    return RMSE(mat1, mat2Prime)



def nothing(x):
    '''dummy function'''
    pass

if __name__ == "__main__":
    cv2.namedWindow("points", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Center X", "points", centerSlam[0]*100, 20000, nothing)
    cv2.createTrackbar("Center Y", "points", centerSlam[1]*100, 20000, nothing)
    cv2.createTrackbar("Rotation", "points", rotateSlam*100   , 36000  , nothing)
    cv2.createTrackbar("Reflection", "points", flipSlam, 1, nothing)
    cv2.createTrackbar("Scale(%)", "points", int(scaleSlam*100)*100, 100000, nothing)
    cv2.createTrackbar("Squish(%X)", "points", int(squishSlam[0]*100)*100, 100000, nothing)
    cv2.createTrackbar("Squish(%Y)", "points", int(squishSlam[1]*100)*100, 100000, nothing)



    fname = "gt-husky-indoor-cutoff.csv"            #groundTruth
    fname2 = "msckf_poses_cutoff.csv"               #estimate
    pts = transformPoints(readCsv(fname))


    pts2 = readCsv(fname2)

    run = True

    while run: #loop through, redraw path for dunamic resizing
        im = numpy.zeros((height, width, 3), numpy.uint8)
        im[:,:] = (255,255,255)
        plotPoints(pts, im)

        #update
        centerSlam[0] = cv2.getTrackbarPos('Center X','points')* resScalar/100.0
        centerSlam[1] = cv2.getTrackbarPos('Center Y','points')* resScalar/100.0
        rotateSlam    = cv2.getTrackbarPos('Rotation','points')/100.0
        flipSlam      = True if cv2.getTrackbarPos('Reflection','points')==1 else False
        scaleSlam     = cv2.getTrackbarPos('Scale(%)','points')/10000.0
        squishSlam[0] = cv2.getTrackbarPos('Squish(%X)','points')/10000.0
        squishSlam[1] = cv2.getTrackbarPos('Squish(%Y)','points')/10000.0

        #transform points
        pts2Prime = transformPoints(pts2, centerSlam, rotateSlam, scaleSlam, flipSlam, squishSlam)

        cv2.putText(im, "RMSE: {:>0.2f}".format(RMSE(pts2Prime, pts)), (5* resScalar, 5* resScalar), cv2.FONT_HERSHEY_PLAIN, 0.2* resScalar, (0,0,0), resThick)
        cv2.putText(im, "Press Esc to Quit"    , (5* resScalar, 10* resScalar), cv2.FONT_HERSHEY_PLAIN, 0.2* resScalar, (0,0,0), resThick)
        cv2.putText(im, "Press 'o' to Optimize", (5* resScalar, 15* resScalar), cv2.FONT_HERSHEY_PLAIN, 0.2* resScalar, (0,0,0), resThick)
        cv2.putText(im, "All Values x100 for definition"             ,(5*resScalar,height), cv2.FONT_HERSHEY_PLAIN, 0.2* resScalar, (0,0,0), resThick)

        plotPoints(pts2Prime, im, (255,0,0))
        cv2.imshow("points", im)
        k = cv2.waitKey(10) & 0xFF    #checks for key being escape
        if k == 27:
            run= False
        if k == 111:
            cv2.putText(im, "Optimizing..."        , (5* resScalar, 20* resScalar), cv2.FONT_HERSHEY_PLAIN, 0.2* resScalar, (0,0,0), resThick)
            cv2.imshow("points", im)
            k = cv2.waitKey(10) & 0xFF
            #print("Optimizing...")
            x0 = [centerSlam[0], centerSlam[1], rotateSlam, scaleSlam, squishSlam[0], squishSlam[1]] #initial guess from current position
            optResult = minimize(optRMSE, x0, args=(flipSlam, pts, pts2), method="Powell")
            #print(optResult.message, optResult.nit)
            if optResult.success:
                cv2.setTrackbarPos('Center X', 'points',int( optResult.x[0]*100/resScalar))
                cv2.setTrackbarPos('Center Y', 'points',int( optResult.x[1]*100/resScalar))
                cv2.setTrackbarPos('Rotation', 'points',int( optResult.x[2]*100))
                cv2.setTrackbarPos('Scale(%)', 'points',int( optResult.x[3]*10000))
                cv2.setTrackbarPos('Squish(%X)', 'points',int( optResult.x[4]*10000))
                cv2.setTrackbarPos('Squish(%Y)', 'points',int( optResult.x[5]*10000))
    cv2.destroyAllWindows()
