#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32

import roslib
import sys
import cv2
import cv
import numpy as np

center = Point32()
count = 0
class KalmanFilter:

	def __init__(self, A, B, Q, H, R, x, p, z):
		self.A = A
		self.B = B
		self.Q = Q
		self.H = H
		self.R = R
		self.x = x
		self.p = p
		self.z = z

	def predict_update(self):
		size = 1
		u = np.zeros(size)
		xpredict = (self.A)*(self.x) + self.B*u
		ppredict = (self.A*self.p)*np.transpose(self.A) + self.Q
		K = np.divide((ppredict*np.transpose(self.H)), (((self.H*ppredict*np.transpose(self.H)) + self.R)))
		xupdate = xpredict + K*(self.z - self.H*xpredict)
		pupdate = (np.identity(size) - K*self.H)*self.p
		return (xupdate, pupdate)

def segment_colour(frame):
    hsv_roi =  cv2.cvtColor(frame, cv2.cv.CV_BGR2HSV)
    mask_1 = cv2.inRange(hsv_roi, np.array([160, 160,10]), np.array([190,255,255]))
    ycr_roi=cv2.cvtColor(frame,cv2.cv.CV_BGR2YCrCb)
    mask_2=cv2.inRange(ycr_roi, np.array((0.,165.,0.)), np.array((255.,255.,255.)))

    mask = mask_1 | mask_2
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode  = np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,kern_erode)
    mask=cv2.dilate(mask,kern_dilate)
    return mask

def find_blob(blob):
    largest_contour=0
    cont_index=0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area > largest_contour):
            largest_contour=area
            cont_index=idx
    r=(0,0,2,2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])

    return r

def callback(image):
    global xval,yval,xsig,ysig,sig1,sig2,wval,wsig,x_velocity,y_velocity,alt,count,x_prev,y_prev,w_prev,h_prev
    bridge = CvBridge()
    cvFrame = bridge.imgmsg_to_cv2(image, "bgr8")
    rate = rospy.Rate(1)
    mask_red=segment_colour(cvFrame)
    loct=find_blob(mask_red)
    x,y,w,h=loct
    print x,y
    width=max(int(w),int(h))
    print width
    distance=(569*18.1)/width   
    print distance #18.1 is the diameter of the ball in centimetres. Change this if you are using a different red coloured object, 569 is the focal length of front camera you should be able to find this in ardrone_autonomy package under data camera info
    cv2.rectangle(cvFrame, (x, y), (x + w, y + h),(0, 255, 0), 2)
    cv2.circle(cvFrame, ((2*x+w)/2, (2*y+h)/2), 3, (255, 0, 0), -1)
    A = 1.0
    B = 0.0
    Q = 1.0
    H = 1.0
    R = 1.0
    x_guess = 0.0
    p_guess = 1.0
    if(count == 0):
		l1 = KalmanFilter(A,B,Q,H,R,x_guess,p_guess,x)
		x_filtered = l1.predict_update()
		x_prev = x_filtered 
		l2 = KalmanFilter(A,B,Q,H,R,x_guess,p_guess,y)
		y_filtered = l2.predict_update()
		y_prev = y_filtered 
		l3 = KalmanFilter(A,B,Q,H,R,x_guess,p_guess,w)
		w_filtered = l3.predict_update()
		w_prev = w_filtered 
		l4 = KalmanFilter(A,B,Q,H,R,x_guess,p_guess,h)
		h_filtered = l4.predict_update()
		h_prev = h_filtered 
		count = count + 1
    else:
		l1 = KalmanFilter(A,B,Q,H,R,float(x_prev[0]),float(x_prev[1]),x)
		x_filtered = l1.predict_update()
		x_prev = x_filtered 
		l2 = KalmanFilter(A,B,Q,H,R,float(y_prev[0]),float(y_prev[1]),y)
		y_filtered = l2.predict_update()
		y_prev = y_filtered
		l3 = KalmanFilter(A,B,Q,H,R,float(h_prev[0]),float(h_prev[1]),w)
		w_filtered = l3.predict_update()
		w_prev = w_filtered 
		l4 = KalmanFilter(A,B,Q,H,R,float(w_prev[0]),float(w_prev[1]),h)
		h_filtered = l4.predict_update()
		h_prev = h_filtered
	

		
    cv2.circle(cvFrame, ((2*x_filtered[0]+w_filtered[0])/2, (2*y_filtered[0]+h_filtered[0])/2), 3, (10, 10, 255), -1)
    cv2.imshow('img2',cvFrame)
#cv2.imshow('img3',cvFrame2)
    #rate.sleep()
    k = cv2.waitKey(5) & 0xff
    #if k == 27:
    #  break
    #cap = cv2.VideoCapture(0)

   # if not cap.isOpened():
    #    cap.open()


    #cv2.destroyAllWindows()

    #cap.release()

def robotics_ardrone():
    rospy.init_node('robotics_ardrone', anonymous=True)
    #callback()
   

    rospy.Subscriber('/ardrone/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        robotics_ardrone()
    except rospy.ROSInterruptException:
        pass
