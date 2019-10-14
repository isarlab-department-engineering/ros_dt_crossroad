#!/usr/bin/env python
import rospy,sys,cv2,roslib, time, threading
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
from master_node.msg import *
from master_node.srv import *
from geometry_msgs.msg import Twist
import numpy as np

font = cv2.FONT_HERSHEY_COMPLEX

onetime = False
id_node = "stop"
positive_answ = 1

twistmessage = Twist()
followmessage = Follow()
followmessage.id = id_node
lock = False
jump = False

pub = rospy.Publisher('follow_topic', Follow, queue_size=1)
talker = rospy.Publisher("check",Int32,queue_size = 1)
request_lock_service = rospy.ServiceProxy('request_lock',RequestLockService)
release_lock_service = rospy.ServiceProxy('release_lock',ReleaseLockService)
stop_service = rospy.ServiceProxy('stop',StopService)


def stop():
    global onetime #used for repeat this function one time for each crossroad
    if not onetime:
        print("Fermo i motori")
        twistmessage.linear.x=0 #set motorspeed to 0
        twistmessage.linear.z=0
        print(twistmessage)
        followmessage.twist = twistmessage
        pub.publish(followmessage)
        #stop_service(0) #keep motor to 0
        onetime=True
        talker.publish(1) #signal to enable program "check_crossroad.py"

def requestLock():
    global id_node, lock, jump
    if lock:
        stop()
    elif jump:
        jump = False
    else:
        resp = request_lock_service(id_node)
        print(resp)
        if resp:
            lock = True
            stop()
        else:
            msg_shared = rospy.wait_for_message("/lock_shared", Lock)
            checkMessage(msg_shared)

def releaseLock():
    global id_node, lock, onetime
    resp = release_lock_service(id_node)
    lock = False
    onetime = False
    print(resp)

def checkMessage(data):
    global id_node, lock
    if data.id == id_node:
        if data.msg == 1:
            lock = True
        else:
            lock = False
    else:
        msg_shared = rospy.wait_for_message("/lock_shared", Lock)
        checkMessage(msg_shared)

def frame_filter(imgmsg):
    global id_node
    bridge = CvBridge()
    frame = bridge.compressed_imgmsg_to_cv2(imgmsg, "bgr8")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.waitKey(1)
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
    lower_red = np.array([0,98,131])
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

            if y > 150:
                if area > 4000:
                    cv2.putText(frame, "STOP DETECTED",(x,y), font, 1, (0,0,255))
                    if y > 240:
                        try:
                            requestLock() #stop procedure
                        except Exception:
                            pass

    cv2.imshow("Rilevazione linea di stop",frame)



def decision(data):
    dec = data.data
    if dec == 10:
        #talker.publish(0)
        turn_right()
    elif dec == 11:
        #talker.publish(0)
        go_straight()
    elif dec == 12:
        #talker.publish(0)
        turn_left()
    else:
        rospy.loginfo("Tipo di dato in formato non giusto")

def nothing():
    pass


def turn_right():
    now = time.time()
    last_time = time.time()
    tt = 5

    while(last_time-now < tt):
        twistmessage.linear.x=0.1
        twistmessage.linear.z=0
        followmessage.twist = twistmessage
        pub.publish(followmessage)
    releaseLock()


def go_straight():
    now = time.time()
    last_time = time.time()
    tt = 5

    while(last_time-now < tt):
        twistmessage.linear.x=0.1
        twistmessage.linear.z=0
        followmessage.twist = twistmessage
        pub.publish(followmessage)
    releaseLock()

def turn_left():
    now = time.time()
    last_time = time.time()
    tt = 5

    while(last_time-now < tt):
        twistmessage.linear.x=0.1
        twistmessage.linear.z=0
        followmessage.twist = twistmessage
        pub.publish(followmessage)
    releaseLock()
  

def main_function():
    rospy.init_node('stop_and_crossing',anonymous=True)
    rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, frame_filter)
    rospy.Subscriber("/svolta", Int32, decision)
    #Release on shutdown
    rospy.on_shutdown(releaseLock)
    rospy.spin()

if __name__=='__main__':
    main_function()
