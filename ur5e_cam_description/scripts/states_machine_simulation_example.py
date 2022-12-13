#!/usr/bin/env python3

# import FK
import rospy
import roslaunch
import smach
from smach_ros import MonitorState, ServiceState, IntrospectionServer
import threading
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

import pyrealsense2 as rs


# define state Camera
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['Success','Failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Init')
        try:
            rospy.loginfo("Initialitating the system")

            rospy.loginfo("¡The system has initialitated correctly!")
            return 'Success'
        except Exception as ex:
            rospy.logerr(ex)
            return 'Failed'


# define state Moving
class Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['Success','Failed'])
        self.data = None
        self.get_data_subscriber = None
        self.cv_image = None
        self.bounding_box = []
        self.bounding_box_sorted = []
        self.get_data_event = threading.Event()

    def get_data_callback(self, data):
        self.data = data
        self.bounding_box = []
        for self.i in (self.data.bounding_boxes):
            self.bounding_box.append(self.i)
            print("\nClass: ")
            print(self.i.Class)
            print("\nEverything: ")
            print(self.i)
        # rospy.sleep(15)
        self.get_data_subscriber.unregister()
        self.get_depth_subscriber.unregister()
        self.get_data_event.set()

    def get_depth_callback (self, data):
        try:
            rospy.loginfo("receiving video frame")
            cvbrd = CvBridge()
            self.cv_image = cvbrd.imgmsg_to_cv2(data)
            # cv.imshow("camera", self.cv_image)
            # cv.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)

    def get_realxyz_callback(self, data):
        print(data)
        self._intrinsics = rs.intrinsics()
        self._intrinsics.width = data.width
        self._intrinsics.height = data.height
        self._intrinsics.ppx = data.K[2]
        self._intrinsics.ppy = data.K[5]
        self._intrinsics.fx = data.K[0]
        self._intrinsics.fy = data.K[4]
        #self._intrinsics.model = data.distortion_model
        self._intrinsics.model  = rs.distortion.none
        self._intrinsics.coeffs = [i for i in data.D]
        result = rs.rs2_deproject_pixel_to_point(self._intrinsics, [self.x_med, self.y_med], self.z_med)
        rospy.loginfo("Coordenadas: " + str(result))
        self.get_realxyz_subscriber.unregister()


    def execute(self, userdata):
        rospy.loginfo('Executing state Moving')

        try:
            rospy.loginfo("Moving to view pose")
            rospy.loginfo("Ejecutando Yolo")
            ruta_yolo = "/home/alberto/tfm_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch"
            uuid_yolo = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid_yolo)
            launch_yolo = roslaunch.parent.ROSLaunchParent(uuid_yolo, [ruta_yolo])
            launch_yolo.start()

            # rospy.sleep(15)

            self.get_data_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.get_data_callback, queue_size=1) #Seguir aquí (hacer función feedback)
            self.get_depth_subscriber = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.get_depth_callback, queue_size=1) #Seguir aquí (hacer función feedback)

            while True:
                self.get_data_event.wait(0.1)
                if self.get_data_event.is_set():
                    self.get_data_event.clear()
                    break
                if rospy.is_shutdown():
                    return 'Failed'
            
            rospy.logwarn('Closing Yolo launch process')

            launch_yolo.shutdown()

            # To return a new list, use the sorted() built-in function...
            # self.bounding_box_sorted = sorted(self.bounding_box, key=Class x: x.count, reverse=True)
            for i in (self.bounding_box):
                self.x_med = round((self.i.xmax + self.i.xmin)/2)
                self.y_med = round((self.i.ymax + self.i.ymin)/2)
                self.z_med = (self.cv_image[self.x_med,self.y_med])/1000

                # rospy.loginfo("Dimensión cv_image: " + str(self.cv_image.shape[:2]))
                # rospy.loginfo("Distance: " + str(self.z_med))
                # focal = 1.88/1000
                # self.perp = (self.z_med)/((1+((((self.x_med-640)**2)+((self.y_med-480)**2))/(focal**2)))**(1/2))
                # self.z_real = self.perp
                # self.x_real = self.perp*((self.x_med-640)/focal)
                # self.y_real = self.perp*((self.y_med-480)/focal)
                # rospy.loginfo("Coordenadas: " + str(self.x_real) + ", " + str(self.y_real) + ", " + str(self.z_real))

                self.get_realxyz_subscriber = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.get_realxyz_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                
                self.cv_image2 = cv.cvtColor(self.cv_image, cv.COLOR_GRAY2RGB)
                print((self.cv_image[self.x_med,self.y_med]))
                cv.circle(self.cv_image2,(self.x_med,self.y_med),5,(0,0,0),-1)
                plt.imshow(self.cv_image)
                cv.imshow("camera", self.cv_image2)
                cv.waitKey(0)
                plt.show()
                cv.destroyAllWindows()


            rospy.loginfo('Esperando 20 segundos para repetir el proceso')
            rospy.sleep(20)

            return 'Success'
        except Exception as ex:
            rospy.logerr(ex)
            return 'Failed'


# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', Init(), 
                                transitions={'Success':'MOVING', 
                                            'Failed':'INIT'})
        smach.StateMachine.add('MOVING', Moving(), 
                                transitions={'Success':'INIT',
                                            'Failed':'MOVING'})

    # Create and start the introspection server
    sis = IntrospectionServer('strat', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()