#!/usr/bin/env python
import rospy,sys,cv2,roslib
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from fiducial_msgs.msg import *
import random

#define aruco vocabolary list
aruco_dict = {'sem_fr':2, 'sem_fl':0, 'sem_lr':1, 'right':3, 'left':4, 'front':5}

cross_dict = {'right':10, 'front':11, 'left':12}

font = cv2.FONT_HERSHEY_COMPLEX

last_signal_id = None

talker = rospy.Publisher("svolta",Int32,queue_size = 1)

foud_green = False



def callback(data):
    global last_signal_id

    fiducial_array = data.fiducials

    if fiducial_array == []:
        pass
        #rospy.loginfo('non vedo nulla')
    else:
        for fiducial in fiducial_array:
            fiducial_id = fiducial.fiducial_id
            if(fiducial_id == aruco_dict["right"] or fiducial_id == aruco_dict["left"] or fiducial_id == aruco_dict["front"]):
                last_signal_id = fiducial_id
                #rospy.loginfo("Visto segnale, aggiorno")


def take_decision(data):
    global last_signal_id, foud_green

    camera_topic = "/raspicam_node/image/compressed"
    semaphore_detected = False
    semaphore_id = None
    semaphore_vert = None

    foud_green = False

    resize_factor = 10

    rospy.loginfo("Stop Detected, cerco semaforo")


    #while(not semaphore_detected):
    while(not foud_green):
        fiducial_array = rospy.wait_for_message("/fiducial_vertices", FiducialArray)

        fiducial_array = fiducial_array.fiducials

        if fiducial_array == []:
            pass
            #rospy.loginfo('non vedo nulla')
        else:
            for fiducial in fiducial_array:
                fiducial_id = fiducial.fiducial_id
                if(fiducial_id == aruco_dict["sem_fr"] or fiducial_id == aruco_dict["sem_fl"] or fiducial_id == aruco_dict["sem_lr"]):
                    semaphore_id = fiducial_id
                    semaphore_vert = [fiducial.x0,fiducial.y0,fiducial.x1,fiducial.y1,fiducial.x2,fiducial.y2,fiducial.x3,fiducial.y3]
                    semaphore_detected = True


        if(semaphore_detected):
            rospy.loginfo("Semaforo trovato, aspetto il verde")
            image = rospy.wait_for_message(camera_topic, CompressedImage)
            bridge = CvBridge()
            frame = bridge.compressed_imgmsg_to_cv2(image, "bgr8")

            window = int(semaphore_vert[5]) - int(semaphore_vert[1])

            crop_image = frame[int(semaphore_vert[1])-window:int(semaphore_vert[1]), int(semaphore_vert[0]):int(semaphore_vert[4])]
            resized = cv2.resize(crop_image, (crop_image.shape[1]*resize_factor,crop_image.shape[0]*resize_factor))

            print("here")

            foud_green = check_semaphore(resized)

    to_send = 0

    if (semaphore_id == aruco_dict["sem_fr"]):
        print("here")
        if(last_signal_id == aruco_dict["right"]):
            to_send = cross_dict["right"]
        elif(last_signal_id == aruco_dict["front"]):
            to_send = cross_dict["front"]
        else:
            rand = random.random
            if(rand<0.5):
                to_send = cross_dict["right"]
            else:
                to_send = cross_dict["front"]

    elif (semaphore_id == aruco_dict["sem_fl"]):
        if(last_signal_id == aruco_dict["left"]):
            to_send = cross_dict["left"]
        elif(last_signal_id == aruco_dict["front"]):
            to_send = cross_dict["front"]
        else:
            rand = random.random
            if(rand<0.5):
                to_send = cross_dict["left"]
            else:
                to_send = cross_dict["front"]

    elif (semaphore_id == aruco_dict["sem_lr"]):
        if(last_signal_id == aruco_dict["left"]):
            to_send = cross_dict["left"]
        elif(last_signal_id == aruco_dict["right"]):
            to_send = cross_dict["right"]
        else:
            rand = random.random
            if(rand<0.5):
                to_send = cross_dict["left"]
            else:
                to_send = cross_dict["right"]

    else:
        rospy.loginfo("Somthing Wrong with Semaphore")

    talker.publish(to_send)

def check_semaphore(image):
    global foud_green


    output = image.copy()

    frame = image

    # grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # brightmask = cv2.threshold(grey, 100, 255, cv2.THRESH_BINARY)[1]

    # bmask = brightmask==0


    # frame[bmask] = 0

    ## convert to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Apply a red mask for stop detection
    #uncomment this code for use trackbars
    # cv2.namedWindow("Trackbars")
    # cv2.createTrackbar("L-H", "Trackbars", 0, 255, nothing)
    # cv2.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
    # cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
    # cv2.createTrackbar("U-H", "Trackbars", 255, 255, nothing)
    # cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
    # cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)
    # l_h = cv2.getTrackbarPos("L-H", "Trackbars")
    # l_s = cv2.getTrackbarPos("L-S", "Trackbars")
    # l_v = cv2.getTrackbarPos("L-V", "Trackbars")
    # u_h = cv2.getTrackbarPos("U-H", "Trackbars")
    # u_s = cv2.getTrackbarPos("U-S", "Trackbars")
    # u_v = cv2.getTrackbarPos("U-V", "Trackbars")
    # lower_red = np.array([l_h,l_s,l_v])
    # upper_red = np.array([u_h,u_s,u_v])
    #comment the next two records while trackbar is uncomment
    lower_red = np.array([0,0,190])
    upper_red = np.array([255,255,255])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    #for filter image
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask,kernel)

    #cv2.imshow("Mask",mask) #uncomment for view the mask filtering
    cv2.waitKey(1)
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #find contours on binary mask
    maxArea = 0
    bestContour = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True) #roximatising polygonal curves
        if area > maxArea: #Calculate max rectangular and best contour
            bestContour = approx
            maxArea = area
        cv2.drawContours(frame, [approx], 0, (0,0,255),1)
        if bestContour is not None:
            x,y,z,t = cv2.boundingRect(bestContour) #build rectangles on best contour
            cv2.rectangle(frame,(x,y),(x+z,y+t),(0,255,0),2)

            if area > 30:
                if(y>(frame.shape[1]/2)):
                    cv2.putText(frame, "GREEN LIGHT",(x,y), font, 1, (0,0,255))
                    foud_green = True
                else:
                    cv2.putText(frame, "RED LIGHT",(x,y), font, 1, (0,0,255))
                # if y > 240:
                #     try:
                #         requestLock() #stop procedure
                #     except Exception:
                #         pass




    # mask = cv2.inRange(hsv, (36, 25, 25), (70, 255,255))

    # ## slice the green
    # imask = mask>0
    # green = np.zeros_like(frame, np.uint8)
    # green[imask] = frame[imask]


    
    # cimg = cv2.cvtColor(green, cv2.COLOR_HSV2BGR)
    
    # cimg = cv2.blur(cimg, (6,6))

    # cimg = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)
    # # cimg = cv2.erode(thresh, None, iterations=2)
    # # cimg = cv2.dilate(cimg, None, iterations=4)
    # k = np.ones((50,50),np.uint8)
    # cimg = cv2.morphologyEx(cimg, cv2.MORPH_TOPHAT,  k)
    # cimg = cv2.threshold(cimg, 20, 255, cv2.THRESH_BINARY)[1]

    # #cimg = cv2.blur(cimg, (6,6))

    # #edges = cv2.Canny(cimg,200,220)

    # circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1, 20, param1=200, param2=5, minRadius=30, maxRadius=100)

    # if circles is not None:
    #     print("Found Green Circle")
    #     circles_to_find = circles_to_find + 1
    #     circles = np.uint16(np.around(circles))
    #     for i in circles[0, :]:
    #          cv2.circle(output, (i[0], i[1]), i[2], (255, 255, 255), 20)
    #          cv2.circle(output, (i[0], i[1]), 2, (255, 255, 255), 30)


    # if(circles_to_find >= circles_thresh):
    #     foud_green = True
    # cv2.imshow('filtered', cimg)
    # cv2.imshow('detected circles', output)
    # cv2.waitKey(1)

    cv2.imshow('Rilevazione rosso', frame)
    cv2.waitKey(1)

    if foud_green:
        cv2.destroyAllWindows()
        rospy.loginfo("Luce verde trovata")

    return foud_green


def main_function():
    rospy.init_node('signal_semaphore',anonymous=True)
    rospy.Subscriber('/fiducial_vertices', FiducialArray, callback)
    rospy.Subscriber("/check", Int32, take_decision)
    rospy.spin()

def nothing():
    pass

if __name__=='__main__':
    main_function()
