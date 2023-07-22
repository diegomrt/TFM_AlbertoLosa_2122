#!/usr/bin/env python3

import FK
import IK_moveJ
import IK_moveL
import ur5_grippers as gripper
import serv_pinza

import rospy
import roslaunch
import smach
from smach_ros import MonitorState, ServiceState, IntrospectionServer
import threading
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo
import tf

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

from math import pi

import pyrealsense2 as rs

from defects import circulo_deforme, circulo_centrado, circulos_base

## Variable del script para saber si estamos en simulación o no
simulacion = 0


# define state Camera
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['Success','Failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Init')
        try:
            rospy.loginfo("Initialitating the system")

            rospy.loginfo("Moving to home pose")
            
            if (simulacion == 1):
                FK.main(0, -90, 90, -90, -90, -90) # Para vgc10
                for topic in rospy.get_published_topics():
                    if (topic[0] == '/camera/color/image_raw'):
                        rospy.loginfo("¡The system has initialitated correctly!")
                        return 'Success'
            else:
                FK.main(90, -90, 90, -90, -90, 0)
                for topic in rospy.get_published_topics():
                    if (topic[0] == '/camera/depth/image_rect_raw'):
                        rospy.loginfo("¡The system has initialitated correctly!")
                        return 'Success'

            rospy.logerr("The system cannot initialitate correctly! The camera is not publishing")
            rospy.sleep(5)
            return 'Failed'
        except Exception as ex:
            rospy.logerr("The system cannot initialitate correctly!")
            rospy.logerr(ex)
            rospy.sleep(5)
            return 'Failed'


# define state Detection
class Detection(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['Success','Failed'],
                                    output_keys=['bounding_box_out',
                                                'image_out'])
        self.data = None
        self.get_data_subscriber = None
        self.bounding_box = []
        self.bounding_box_sorted = []
        self.get_data_event = threading.Event()
        self.resultado = None
        self.cv_image = None

    def get_data_callback(self, data):
        try:
            self.data = data
            self.bounding_box = []
            rospy.loginfo("\nDetected the following elements:")
            for element in (self.data.bounding_boxes):
                rospy.loginfo("\nClass: " + element.Class)
                self.bounding_box.append(element)
            self.get_data_subscriber.unregister()
            self.get_data_subscriber = None
            self.get_depth_subscriber.unregister()
            self.get_depth_subscriber = None
            self.get_data_event.set()
        except CvBridgeError as e:
            rospy.logerr("Error en data_callback" + e)

    def get_depth_callback (self, data):
        try:
            # rospy.loginfo("receiving video frame")
            cvbrd = CvBridge()
            self.cv_image = cvbrd.imgmsg_to_cv2(data)
            # cv.imshow("camera1", self.cv_image)
            # cv.waitKey(0)
        except CvBridgeError as e:
            rospy.logerr("Error en depth_callback" + e)
        

    def execute(self, userdata):
        rospy.loginfo('Executing state Detection')
        self.object_done = False

        try:
            rospy.loginfo("Moving to view pose")
            if (simulacion == 1):
                FK.main(0, -90, 90, -90, -90, -90) # Para vgc10
                ruta_yolo = "/home/alberto/tfm_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch"
            else:
                FK.main(90, -90, 90, -90, -90, 0)
                ruta_yolo = "/home/alberto/tfm_ws/src/darknet_ros/darknet_ros/launch/yolo_v3-custom.launch"
            rospy.loginfo("Ejecutando Yolo")
            uuid_yolo = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid_yolo)
            launch_yolo = roslaunch.parent.ROSLaunchParent(uuid_yolo, [ruta_yolo])
            launch_yolo.start()

            # rospy.sleep(15)

            self.get_data_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.get_data_callback, queue_size=1) #Seguir aquí (hacer función feedback)
            if (simulacion == 1):
                self.get_depth_subscriber = rospy.Subscriber('/camera/depth/image_raw', Image, self.get_depth_callback, queue_size=1) #Seguir aquí (hacer función feedback)
            else:
                self.get_depth_subscriber = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.get_depth_callback, queue_size=1) #Seguir aquí (hacer función feedback)

            while True:
                self.get_data_event.wait(1)
                if self.get_data_event.is_set():
                    self.get_data_event.clear()
                    userdata.bounding_box_out = self.bounding_box
                    userdata.image_out = self.cv_image
                    break
                if rospy.is_shutdown():
                    return 'Failed'
            
            rospy.logwarn('Closing Yolo launch process')

            launch_yolo.shutdown()

            return 'Success'
        except Exception as ex:
            rospy.logerr(ex)
            return 'Failed'

# define state Picking
class Picking(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['Success','Failed'],
                                    input_keys=['bounding_box_in',
                                                'image_in'])
        self.cv_image = None
        self.objeto = None
        self.obj_actual = None
        self.orientacion = None
        self.offset = None
        self.cv_image_color = None
        self.frame_tomado = False

    def execute(self, userdata):
        rospy.loginfo('Executing state Picking')
        self.cv_image = userdata.image_in
        try:
            for elem in (userdata.bounding_box_in):
                self.obj_actual = elem
                rospy.loginfo("Going and taking the object %s" % elem.Class )
                self.object_done = False
                self.x_med = round((elem.xmax + elem.xmin)/2)
                self.y_med = round((elem.ymax + elem.ymin)/2)
                if (simulacion == 1):
                    self.z_med = (self.cv_image[int((self.y_med)),int((self.x_med))])/1000
                else:
                    # self.z_med = (self.cv_image[int((self.y_med*(2/3))),int((self.x_med)*(2/3))])/1000
                    self.z_med = (self.cv_image[int((self.y_med)), int((self.x_med))])/1000
                
                rospy.loginfo('Z medida: ' + str(self.z_med))

                self.get_realxyz_subscriber = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.get_realxyz_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                

                while self.object_done == False:
                    rospy.sleep(10)

            return 'Success'
        except Exception as ex:
            rospy.logerr(ex)
            return 'Failed'

    def get_color_callback (self, data):
        try:
            rospy.loginfo("receiving video frame")
            cvbrd = CvBridge()
            self.cv_image_color = cvbrd.imgmsg_to_cv2(data)
            # cv.imshow("camera1", self.cv_image)
            # cv.waitKey(0)
            self.frame_tomado = True
        except CvBridgeError as e:
            rospy.logerr("Error en color_callback" + e)

    def get_realxyz_callback(self, data):
        self._intrinsics = rs.intrinsics()
        self._intrinsics.width = data.width
        self._intrinsics.height = data.height
        self._intrinsics.ppx = data.K[2]
        self._intrinsics.ppy = data.K[5]
        self._intrinsics.fx = data.K[0]
        self._intrinsics.fy = data.K[4]
        #self._intrinsics.model = data.distortion_model
        self._intrinsics.model  = rs.distortion.none
        if (simulacion == 1):
            self._intrinsics.coeffs = [i for i in [0.0, 0.0, 0.0, 0.0, 0.0]]
        else:
            self._intrinsics.coeffs = [i for i in data.D]
        self.result = rs.rs2_deproject_pixel_to_point(self._intrinsics, [self.x_med, self.y_med], self.z_med)
        rospy.loginfo("Coordenadas del objeto en píxeles: " + str(self.result))
        self.get_realxyz_subscriber.unregister()

        # Actualizamos la transformada
        self.pos_publisher = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        if (simulacion == 1):
            self.objeto = "frisbee"
        else:
            self.objeto = "escuadra"
        while ((not rospy.is_shutdown()) and self.object_done == False):
            if (self.obj_actual.Class == self.objeto):
                self.pos_publisher.sendTransform((self.result[0],
                                            self.result[1],
                                            self.result[2]), tf.transformations.quaternion_from_euler(0, 0, pi),
                                            rospy.Time.now(), "objeto", "camera_color_optical_frame")
                
                try:
                    (trans, rotation) = self.listener.lookupTransform('base_link', 'objeto', rospy.Time(0))
                    rospy.loginfo("La transformada del objeto se encuentra en: %f(x), %f(y), %f(z)", trans[0], trans[1], trans[2])

                    # Offset simulación/real
                    if (simulacion == 1):
                        self.offset = 0
                        ## Para ver la pieza por el lado más cercano
                        if (trans[1] < 0):
                            # self.orientacion = -1
                            self.orientacion = -pi/2
                            self.acercamiento = 0.03
                        else:
                            # self.orientacion = 1
                            self.orientacion = pi/2
                            self.acercamiento = -0.03
                    else:
                        self.offset = pi/2
                        ## Para ver la pieza por el lado más cercano
                        if (trans[0] < 0):
                            # self.orientacion = -1
                            self.orientacion = pi/2
                            self.acercamiento = 0.029
                        else:
                            # self.orientacion = 1
                            self.orientacion = -pi/2
                            self.acercamiento = -0.029
                    # En el robot real creo que sería (pi, pi/2, 0) y (0, pi/2, 0)
                    # Para unificar métodos se podría poner 0 +-pi/2 en simulación y pi/2 +-pi/2 en real

                    #### Nos acercamos lateralmente a la pieza para comprobar si hay defectos
                    self.pos_publisher.sendTransform((trans[0],
                                            trans[1],
                                            0.22), tf.transformations.quaternion_from_euler(self.orientacion + self.offset, pi/2, 0),
                                            rospy.Time.now(), "objeto2", "base_link")   ### Para ir perpend al suelo

                    self.resultado = input ("Comprueba en rviz la posición de la herramienta, ¿seguro que quiere acudir a esa posición? (y or n)")

                    if self.resultado == "y":
                    
                        rospy.sleep(0.3)

                        if (simulacion == 1):
                            x ,y, z = IK_moveL.main_euler_tcp(pi, 0, 0, trans[0]+self.acercamiento, trans[1], 0.25)  ### Punto de acercamiento
                            x ,y, z = IK_moveL.main_euler_tcp(self.orientacion + self.offset, pi/2, 0, trans[0]+self.acercamiento, trans[1]+self.acercamiento, 0.165)  ### Para ir con la cámara enfocando la pieza lateralmente
                            self.get_color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.get_color_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                            while self.frame_tomado == False:
                                rospy.sleep(1)
                                rospy.loginfo("Waiting for frame")
                            self.get_color_subscriber.unregister()
                            # cv.imshow("imagen", self.cv_image_color)
                            # cv.waitKey(0)
                            # cv.destroyAllWindows()
                            print(circulo_deforme.main(self.cv_image_color))
                            self.frame_tomado = False
                            rospy.sleep(5)
                            x ,y, z = IK_moveJ.main_euler_tcp(pi, 0, 0, trans[0]-0.115, trans[1]+0.03, 0.03)  ### Para ver la pieza de frente
                            self.get_color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.get_color_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                            while self.frame_tomado == False:
                                rospy.sleep(1)
                            ## Meter funciones de visión
                            self.get_color_subscriber.unregister()
                            self.frame_tomado = False
                        else:
                            print(IK_moveL.main_euler_tcp(pi, 0, pi/2, trans[0]+self.acercamiento, trans[1], 0.25))  ### Punto de acercamiento
                            x ,y, z = IK_moveL.main_euler_tcp(self.orientacion + self.offset, pi/2, 0, trans[0]+self.acercamiento, trans[1]+self.acercamiento, 0.165)  ### Para ir con la cámara enfocando la pieza lateralmente
                            self.get_color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.get_color_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                            while self.frame_tomado == False:
                                rospy.sleep(1)
                                rospy.loginfo("Waiting for frame")
                            self.get_color_subscriber.unregister()
                            # cv.imshow("imagen", self.cv_image_color)
                            # cv.waitKey(0)
                            # cv.destroyAllWindows()
                            try:
                                rospy.loginfo("Primera función: se comprueba la circularidad del circulo central")
                                print(circulo_deforme.main(self.cv_image_color))
                                rospy.loginfo("Segunda función: se comprueba el centrado del circulo central")
                                print(circulo_centrado.main(self.cv_image_color))
                            except Exception as ex:
                                rospy.logerr(ex)
                                return 'Failed'
                            self.frame_tomado = False
                            rospy.sleep(5)
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, trans[0]+self.acercamiento, trans[1], 0.25)
                            x ,y, z = IK_moveJ.main_euler_tcp(pi, 0, 0, trans[0]-0.115, trans[1]+0.03, 0.03)  ### Para ver la pieza de frente
                            self.get_color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.get_color_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                            while self.frame_tomado == False:
                                rospy.sleep(1)
                            self.get_color_subscriber.unregister()
                            try:
                                print(circulos_base.main(self.cv_image_color))
                            except Exception as ex:
                                rospy.logerr(ex)
                                return 'Failed'
                            self.frame_tomado = False

                        rospy.sleep(5)

                        self.object_done = True

                    else:
                        rospy.logwarn("Se ha cancelado el acudir a recoger el objeto")
                        self.object_done = True
                        break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("Error al publicar transformada TF")
                    rospy.sleep(1)
                    continue
                except Exception as ex:
                    rospy.logerr(ex)
                    return 'Failed'


            else:
                self.pos_publisher.sendTransform((self.result[0],
                                            self.result[1],
                                            self.result[2]), tf.transformations.quaternion_from_euler(0, 0, pi),
                                            rospy.Time.now(), "objeto", "camera_color_optical_frame")
                try:
                    (trans, rotation) = self.listener.lookupTransform('base_link', 'objeto', rospy.Time(0))
                    rospy.loginfo("La transformada del objeto se encuentra en: %f(x), %f(y), %f(z)", trans[0], trans[1], trans[2])
                    ##### DEBUG ######
                    # rospy.logdebug("Quaternion del objeto se encuentra en: ", str(rotation))
                    # rospy.logdebug("La transformada a euler del objeto se encuentra en: ", str(tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])))
                    ##################


                    #### Para ir con la misma orientación que la recta que une la cámara y el objeto
                    # self.pos_publisher.sendTransform((trans[0],
                    #                         trans[1],
                    #                         trans[2]), rotation,
                    #                         rospy.Time.now(), "objeto2", "base_link")  ### Vamos con la misma orientación que la recta que une la camara y el objeto

                    #### Para ir perpendicular al suelo
                    self.pos_publisher.sendTransform((trans[0],
                                            trans[1],
                                            trans[2]), tf.transformations.quaternion_from_euler(pi, 0, 0),
                                            rospy.Time.now(), "objeto2", "base_link")   ### Para ir perpend al suelo

                    self.resultado = input ("Comprueba en rviz la posición del objeto, ¿seguro que quiere acudir a esa posición? (y or n)")


                    if self.resultado == "y":
                    
                        rospy.sleep(2)

                        # IK_moveJ.main_quaternion(rotation, trans[0], trans[1], trans[2])  ### Vamos con la misma orientación que la recta que une la camara y el objeto
                        # IK_moveJ.main_quaternion(tf.transformations.quaternion_from_euler(pi, 0, 0), trans[0], trans[1], trans[2])  ### Para ir perpend al suelo con wrist_3
                        x ,y, z = IK_moveJ.main_euler_tcp(pi, 0, 0, trans[0], trans[1], trans[2])  ### Para ir perpend al suelo con tcp
                        self.pos_publisher.sendTransform((x,
                                            y,
                                            z), tf.transformations.quaternion_from_euler(pi, 0, 0),
                                            rospy.Time.now(), "punto_aprox", "base_link")   ### Para ir perpend al suelo

                        rospy.sleep(5)

                        rospy.loginfo("Activando pinza de vacío")   #### Hay que ver como activarla ######
                        if (simulacion == 1):
                            gripper.trigger_gripper(True)
                        else:
                            serv_pinza.turn(1)
                        rospy.sleep(2)
                        rospy.loginfo("Paso por punto intermedio")
                        IK_moveJ.main_euler_tcp(pi, 0, 0, trans[0], trans[1], 0.3)  ### Para ir perpend al suelo con tcp
                        rospy.loginfo("Dejando objeto")
                        FK.main(90, -85, 109, -113, -90, 0)
                        # FK.main(86, -85, 109, -113, -90, -180) ## Donde lo dejamos?
                        rospy.loginfo("Desactivando pinza de vacío")
                        if (simulacion == 1):
                            gripper.trigger_gripper(False)
                        else:
                            serv_pinza.turn(0)
                        rospy.sleep(10)
                        rospy.loginfo("Terminado movimiento")
                    else:
                        rospy.logwarn("Se ha cancelado el acudir a recoger el objeto")
                    self.object_done = True
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr("Error al publicar transformada TF")
                    rospy.sleep(1)
                    continue

            rospy.sleep(1)
            self.get_realxyz_subscriber.unregister()



# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[])
    sm.userdata.bounding_boxes = []
    sm.userdata.image = None

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', Init(), 
                                transitions={'Success':'DETECTION', 
                                            'Failed':'INIT'})
        smach.StateMachine.add('DETECTION', Detection(), 
                                transitions={'Success':'PICKING',
                                            'Failed':'DETECTION'},
                                remapping={'bounding_box_out':'bounding_boxes',
                                            'image_out':'image'})
        smach.StateMachine.add('PICKING', Picking(), 
                                transitions={'Success':'DETECTION',
                                            'Failed':'DETECTION'},
                                remapping={'bounding_box_in':'bounding_boxes',
                                            'image_in':'image'})

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