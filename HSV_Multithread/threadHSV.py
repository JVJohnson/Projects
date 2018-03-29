#experiments with multiprocessing
#stolen from tsack overflow user P.ro @
#https://stackoverflow.com/questions/42284122/opencv-python-multi-threading-for-live-facial-recognition


import cv2
import numpy as np

from multiprocessing import Process, Queue
import time


class HSV_Finder(Process):
    def __init__(self, frame_queue, output_queue):
        Process.__init__(self)
        self.frame_queue    = frame_queue
        self.output_queue   = output_queue
        self.running        = True

    def get_frame(self):
        if not self.frame_queue.empty():
            return True, self.frame_queue.get()
        else:
            return False, None

    def stopProcess(self):
        self.running = False

    def find_hsv(self, frame):
        im, lowerHSV, upperHSV = frame
        hsv = cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,lowerHSV, upperHSV)

        mask = cv2.GaussianBlur(mask, (51, 51), 0)
        mask = cv2.erode(mask, (5,5), iterations=20)
        im2, Contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        if len(Contours) >0:
            Contours = sorted(Contours, key=cv2.contourArea)
            cont = Contours[0]
            M = cv2.moments(cont)                 #creates moments of contour
            cX = int(M["m10"]/M["m00"])                 #uses moments for the circle
            cY = int(M["m01"]/M["m00"])

            cv2.drawContours(im, [cont], -1, (255,0,255), 2)#draws the contours
            cv2.circle(im, (cX, cY), 12 ,(50, 50, 255), -1) #draws the circle
            if self.output_queue.full():
                self.output_queue.get_nowait()
            else:
                self.output_queue.put(  (im, (cX, cY) )  )


    def run(self):
        while self.running:
            ret, frame = self.get_frame()
            if ret:
                self.find_hsv(frame)


if __name__ == "__main__":
    frame_sum = 0
    init_time = time.time()
    h,s,v, h1, s1, v1 = (100,100,100, 0, 0, 0)

    def put_frame(frame):
        if Input_Queue.full(): 
            Input_Queue.get_nowait()
        else:
            Input_Queue.put(frame)

    def create_frame(cap):
        global h,s,v,h1,s1,v1
        s = cv2.getTrackbarPos('s','Threaded Video')
        h = cv2.getTrackbarPos('h','Threaded Video')
        v = cv2.getTrackbarPos('v','Threaded Video')
        h1 = cv2.getTrackbarPos('h1','Threaded Video')
        s1 = cv2.getTrackbarPos('s1','Threaded Video')
        v1 = cv2.getTrackbarPos('v1','Threaded Video')
    
        lower_blue = np.array([h,s,v])
        upper_blue = np.array([h1,s1,v1])       
        frame = (cap.copy(), lower_blue, upper_blue)

        put_frame(frame)

    def nothing(x): 
        pass
        
    cap = cv2.imread("Python.png", 1)

    threadn = cv2.getNumberOfCPUs()

    threaded_mode = True

    process_list = []
    Input_Queue = Queue(maxsize = 5)
    Output_Queue = Queue(maxsize = 5)

    for x in range((threadn -1)):    
        hsvProcess = HSV_Finder(frame_queue = Input_Queue,output_queue = Output_Queue)
        hsvProcess.daemon = True
        hsvProcess.start()
        process_list.append(hsvProcess)

    ch = cv2.waitKey(1)
    cv2.namedWindow('Threaded Video', cv2.WINDOW_NORMAL)

    cv2.createTrackbar('h1', 'Threaded Video',0,179,nothing)
    cv2.createTrackbar('s1', 'Threaded Video',0,255,nothing)
    cv2.createTrackbar('v1', 'Threaded Video',0,255,nothing)

    cv2.createTrackbar('h', 'Threaded Video',0,179,nothing)
    cv2.createTrackbar('s', 'Threaded Video',0,255,nothing)
    cv2.createTrackbar('v', 'Threaded Video',0,255,nothing)
    while threaded_mode: 

        create_frame(cap)
        
        
        if not Output_Queue.empty():
            result = Output_Queue.get()
            im, c = result
            cv2.imshow('Threaded Video', im)
            ch = cv2.waitKey(5)
            #print("other")
        else: 
            #print("iter")
            cv2.imshow('Threaded Video', cap.copy())
            ch = cv2.waitKey(5)
        if ch == ord(' '):
            threaded_mode = not threaded_mode
        if ch == 27:
            break
    cv2.destroyAllWindows()

    print("Upper H: {:>3}  S: {:>3}   V: {:>3}".format(h1,s1,v1))
    print("Lower H: {:>3}  S: {:>3}   V: {:>3}".format(h,s,v))


