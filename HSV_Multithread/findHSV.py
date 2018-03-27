import cv2
import numpy as np

cap = cv2.imread("Python.png",1)

def nothing(x):
    pass
# Creating a window for later use
cv2.namedWindow('result', cv2.WINDOW_NORMAL)
run = True
# Starting with 100's to prevent error while masking
h,s,v = 100,100,100

# Creating track bar

cv2.createTrackbar('h1', 'result',0,179,nothing)
cv2.createTrackbar('s1', 'result',0,255,nothing)
cv2.createTrackbar('v1', 'result',0,255,nothing)

cv2.createTrackbar('h', 'result',0,179,nothing)
cv2.createTrackbar('s', 'result',0,255,nothing)
cv2.createTrackbar('v', 'result',0,255,nothing)

while(run):

    frame = cap.copy()

    #converting to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
    s = cv2.getTrackbarPos('s','result')
    h = cv2.getTrackbarPos('h','result')
    v = cv2.getTrackbarPos('v','result')
    h1 = cv2.getTrackbarPos('h1','result')
    s1 = cv2.getTrackbarPos('s1','result')
    v1 = cv2.getTrackbarPos('v1','result')

    print("lower = ", h,s,v)
    print("upper = ", h1,s1,v1)

    # Normal masking algorithm
    lower_blue = np.array([h,s,v])
    upper_blue = np.array([h1,s1,v1])

    mask = cv2.inRange(hsv,lower_blue, upper_blue)

    mask = cv2.GaussianBlur(mask, (51, 51), 0)
    mask = cv2.erode(mask, (5,5), iterations=20)
    im2, Contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(Contours) >0:
        Contours = sorted(Contours, key=cv2.contourArea)
        cont = Contours[0]
        M = cv2.moments(cont)                 #creates moments of contour
        cX = int(M["m10"]/M["m00"])                 #uses moments for the circle
        cY = int(M["m01"]/M["m00"])

        cv2.drawContours(frame, [cont], -1, (255,0,255), 2)#draws the contours
        cv2.circle(frame, (cX, cY), 12 ,(50, 50, 255), -1) #draws the circle

    # result = cv2.bitwise_and(frame,frame,mask = mask)

    cv2.imshow('result', frame)

    k = cv2.waitKey(10) & 0xFF
    if k == 27:
        run = False



cv2.destroyAllWindows()
