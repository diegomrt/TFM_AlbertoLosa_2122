#!/usr/bin/env python3

import rospy
import roslaunch
from sensor_msgs.msg import Image, CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

import circulo_deforme, circulo_centrado, circulos_base, circulos_base_prof, orientacion_pieza_bound_box

cvbrd = CvBridge()
frame_depth = None
frame_color = None
bounding_box = []
global suma
suma = 0

get_depth_subscriber = None
get_color_subscriber = None
get_yolo_subscriber = None

def get_depth_callback(data):
    try:
        global suma
        global frame_depth
        global get_depth_subscriber
        frame_depth = cvbrd.imgmsg_to_cv2(data)
        # cv.imshow("camera1", frame_depth)
        # cv.waitKey(0)
        # cv.destroyAllWindows()
        suma = suma + 1
        get_depth_subscriber.unregister()
    except CvBridgeError as e:
        rospy.logerr("Error en depth_callback" + e)

def get_color_callback(data):
    try:
        global suma
        global frame_color
        frame_color = cvbrd.imgmsg_to_cv2(data)
        # cv.imshow("camera1", self.cv_image)
        # cv.waitKey(0)
        suma = suma + 1
    except CvBridgeError as e:
        rospy.logerr("Error en depth_callback" + e)

def get_yolo_callback(data):
    try:
        global bounding_box
        global suma
        global get_yolo_subscriber
        rospy.loginfo("\nDetected the following elements:")
        for element in (data.bounding_boxes):
            rospy.loginfo("\nClass: " + element.Class)
            bounding_box.append(element)
        get_yolo_subscriber.unregister()
        suma = suma + 1
    except CvBridgeError as e:
        rospy.logerr("Error en data_callback" + e)


def listener():

    global frame_color
    global frame_depth
    global bounding_box
    global suma

    global get_depth_subscriber
    global get_yolo_subscriber

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    resultado = 0

    # while (resultado != "y" and resultado != "Y"):

    #     suma = 0

    #     get_depth_subscriber = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, get_depth_callback, queue_size=1)
    #     get_color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, get_color_callback, queue_size=1)

    #     while (suma < 2):
    #         print(suma)
    #         rospy.sleep(0.5)

    #     get_depth_subscriber.unregister()
    #     get_color_subscriber.unregister()

    #     cv.imshow('depth', frame_depth)
    #     cv.imshow('color', frame_color)
    #     cv.waitKey(0)
    #     cv.destroyAllWindows()

    #     cimg = np.copy(frame_color)
    #     img_gray = cv.cvtColor(frame_color,cv.COLOR_BGR2GRAY)

    #     try:

    #         # Calculo de círculos
    #         # circles = cv.HoughCircles(img_gray,cv.HOUGH_GRADIENT_ALT,0.9,40,param1=50,param2=0.9,minRadius=4,maxRadius=0)
    #         circles = cv.HoughCircles(img_gray,cv.HOUGH_GRADIENT,1,100,param1=200,param2=8,minRadius=3,maxRadius=10)
    #         circles = np.uint16(np.around(circles))

    #         # Pintar elementos sobre la imagen
    #         for i in circles[0,:]:
    #             # draw the outer circle
    #             cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    #             # draw the center of the circle
    #             cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

    #         cv.imshow('circulos',cimg)
    #         cv.waitKey(0)
    #         cv.destroyAllWindows()

    #         resultado = input ("¿Buenos círculos? (y or n)")
        
    #     except Exception as ex:
    #         print(ex)


    # orientacion_VA = circulos_base_prof.main(frame_color, frame_depth)
    # print(orientacion_VA)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


    get_depth_subscriber = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, get_depth_callback, queue_size=1)
    get_yolo_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, get_yolo_callback, queue_size=1)

    while (suma < 2):
        print(suma)
        rospy.sleep(0.5)


    print(orientacion_pieza_bound_box.main(frame_depth, bounding_box[0].xmax, bounding_box[0].ymax, bounding_box[0].xmin, bounding_box[0].ymin))

if __name__ == '__main__':
    listener()

