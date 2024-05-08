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

from defects import circulo_deforme, circulo_centrado, circulos_base, orientacion_pieza_bound_box

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
        # Inicializacion de variables
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
            cvbrd = CvBridge()
            self.cv_image = cvbrd.imgmsg_to_cv2(data)
            # DEBUG
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
                FK.main(0, -90, 90, -90, -90, -90)
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
            # For para todos los objetos detectados
            for elem in (userdata.bounding_box_in):

                # Movimiento hasta la posición de inicio
                rospy.loginfo("Moving to start picking pose")
                if (simulacion == 1):
                    FK.main(0, -90, 90, -90, -90, -90)
                else:
                    FK.main(90, -90, 90, -90, -90, 0)

                # Obtención de elemento e inicialización de variables necesarias
                self.obj_actual = elem
                rospy.loginfo("Going and taking the object %s" % elem.Class )
                self.object_done = False
                self.xmax = elem.xmax
                self.xmin = elem.xmin
                self.ymax = elem.ymax
                self.ymin = elem.ymin
                self.x_med = round((elem.xmax + elem.xmin)/2)
                self.y_med = round((elem.ymax + elem.ymin)/2)

                # Cálculo de altura del objeto con imagen de profundidad
                self.z_med = (self.cv_image[int((self.y_med)), int((self.x_med))])/1000
                
                # DEBUG
                # rospy.loginfo('Z medida: ' + str(self.z_med))

                # Inicialización de callback para obtener parámetros de la cam y realizar análisis VA y pick and place
                self.get_realxyz_subscriber = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.get_realxyz_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                
                while self.object_done == False:
                    rospy.sleep(2)

            return 'Success'

        except Exception as ex:
            rospy.logerr(ex)
            return 'Failed'

    def get_color_callback (self, data):
        try:
            rospy.loginfo("receiving video frame")
            cvbrd = CvBridge()
            self.cv_image_color = cvbrd.imgmsg_to_cv2(data)
            # DEBUG
            # cv.imshow("camera1", self.cv_image)
            # cv.waitKey(0)
            self.frame_tomado = True

        except CvBridgeError as e:
            rospy.logerr("Error en color_callback" + e)

    def get_realxyz_callback(self, data):

        # Se obtienen datos de calibración de la cam para obtener posición real del objeto
        self._intrinsics = rs.intrinsics()
        self._intrinsics.width = data.width
        self._intrinsics.height = data.height
        self._intrinsics.ppx = data.K[2]
        self._intrinsics.ppy = data.K[5]
        self._intrinsics.fx = data.K[0]
        self._intrinsics.fy = data.K[4]
        self._intrinsics.model  = rs.distortion.none
        if (simulacion == 1):
            self._intrinsics.coeffs = [i for i in [0.0, 0.0, 0.0, 0.0, 0.0]]
        else:
            self._intrinsics.coeffs = [i for i in data.D]
        
        # Se obtiene la posición real del objeto
        self.result = rs.rs2_deproject_pixel_to_point(self._intrinsics, [self.x_med, self.y_med], self.z_med)
        # DEBUG
        # rospy.loginfo("Coordenadas del objeto desde la camara: " + str(self.result))

        self.get_realxyz_subscriber.unregister()

        # Actualizamos la transformada
        self.pos_publisher = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # Se analiza el tipo de objeto para VA y pick and palce
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
                    # Transformada a base de coordenadas del robot (base_link)
                    (trans, rotation) = self.listener.lookupTransform('base_link', 'objeto', rospy.Time(0))
                    rospy.loginfo("La transformada del objeto se encuentra en: %f(x), %f(y), %f(z)", trans[0], trans[1], trans[2])

                    # Offset simulación/real
                    if (simulacion == 1):
                        self.offset = 0
                        ## Para ver la pieza por el lado más cercano
                        if (trans[1] < 0):
                            self.orientacion = -pi/2
                            self.acercamiento = 0.03
                        else:
                            self.orientacion = pi/2
                            self.acercamiento = -0.03

                    else:
                        self.offset = pi/2
                        ## Para ver la pieza por el lado más cercano
                        if (trans[0] < 0):
                            self.orientacion = pi/2
                            self.acercamiento = 0.035
                            self.acercamiento_x = -0.05
                        else:
                            self.orientacion = -pi/2
                            self.acercamiento = -0.035
                            self.acercamiento_x = 0.05

                    # Variable para discretizar comprobación del estado de la pieza
                    self.check_pieza = True

                    self.resultado = input ("Comprueba en rviz la posición de la herramienta, ¿seguro que quiere acudir a esa posición? (y or n)")

                    if self.resultado == "y":
                    
                        rospy.sleep(0.3)

                        if (simulacion == 1):
                            x ,y, z = IK_moveL.main_euler_tcp(pi, 0, 0, trans[0]+self.acercamiento, trans[1], 0.25)  ### Punto de acercamiento
                            x ,y, z = IK_moveL.main_euler_tcp(0, pi/2, pi, trans[0]-0.029, trans[1]-0.029, 0.165)  ### Para ir con la cámara enfocando la pieza lateralmente

                            self.get_color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.get_color_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                            
                            while self.frame_tomado == False:
                                rospy.sleep(1)
                            self.get_color_subscriber.unregister()
                            
                            # DEBUG
                            # cv.imshow("imagen", self.cv_image_color)
                            # cv.waitKey(0)
                            # cv.destroyAllWindows()

                            # Estudio de la deformidad del circulo central (no modelada la pieza en gazebo)
                            if (circulo_deforme.main(self.cv_image_color) == "mal"):
                                self.check_pieza = False
                                print("El orificio central no posee la circularidad adecuada")
                            else:
                                print("El orificio central está correcto")
                            
                            self.frame_tomado = False
                            rospy.sleep(0.1)

                            ### Ver la pieza desde arriba
                            x ,y, z = IK_moveJ.main_euler_tcp(pi, 0, 0, trans[0]-0.115, trans[1]+0.03, 0.03)

                            self.get_color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.get_color_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                            while self.frame_tomado == False:
                                rospy.sleep(0.2)

                            ## Meter funciones de visión (no modelada la pieza en gazebo)

                            self.get_color_subscriber.unregister()
                            self.frame_tomado = False
                        else:
                            # Estudio de la orientacion de la pieza
                            self.orientacion_VA, recta_hist = orientacion_pieza_bound_box.main(self.cv_image, self.xmax, self.ymax, self.xmin, self.ymin)
                            print("Orientación de la pieza: " , self.orientacion_VA)
                            
                            ### Ver la pieza desde arriba
                            x ,y, z = IK_moveJ.main_euler_tcp(pi, 0, 0, trans[0]-0.115, trans[1]+0.03, 0.03)

                            self.get_color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.get_color_callback, queue_size=1) #Seguir aquí (hacer función feedback)

                            while self.frame_tomado == False:
                                rospy.sleep(0.2)
                            self.get_color_subscriber.unregister()

                            # Estudio de los 4 agujeros de la base
                            try:
                                resultado_cir_base = circulos_base.main(self.cv_image_color)
                                if (resultado_cir_base != "Pieza correcta"):
                                    self.check_pieza = False
                                    print("Desperfecto en los orificios de la base. " + resultado_cir_base )
                                else:
                                    print("Los orificios de la base están correctos")
                            except Exception as ex:
                                self.check_pieza = False
                                rospy.logerr(ex)
                            self.frame_tomado = False

                            # Segun la orientacion, el brazo se coloca en posiciones diferentes para verla de frente
                            if (self.orientacion_VA == "Orientación x"):
                                IK_moveL.main_euler_tcp(pi, 0, pi/2, trans[0]+self.acercamiento, trans[1], 0.25)  ### Punto de acercamiento
                                IK_moveL.main_euler_tcp(self.orientacion + self.offset, pi/2, 0, trans[0]+self.acercamiento_x, trans[1]+self.acercamiento, 0.165)  ### Para ir con la cámara enfocando la pieza lateralmente
                                self.get_color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.get_color_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                            
                            elif (self.orientacion_VA == "Orientación y"):
                                IK_moveL.main_euler_tcp(pi, 0, pi/2, trans[0]+self.acercamiento, trans[1], 0.25)  ### Punto de acercamiento
                                IK_moveL.main_euler_tcp(pi/2, pi/2, 0, trans[0]-0.029, trans[1]-0.029, 0.165)  ### Para ir con la cámara enfocando la pieza lateralmente
                                self.get_color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.get_color_callback, queue_size=1) #Seguir aquí (hacer función feedback)
                            
                            else:
                                rospy.logerr("No se ha podido determinar la orientación de la pieza")
                                self.check_pieza = False
                                break
                            
                            while self.frame_tomado == False:
                                rospy.sleep(0.5)
                            self.get_color_subscriber.unregister()

                            # DEBUG
                            # cv.imshow("imagen", self.cv_image_color)
                            # cv.waitKey(0)
                            # cv.destroyAllWindows()

                            try:
                                # Estudio de la circularidad del circulo central
                                rospy.loginfo("Primera función: se comprueba la circularidad del circulo central")
                                if (circulo_deforme.main(self.cv_image_color) == "bien"):
                                    print("El orificio central está correcto")
                                else:
                                    self.check_pieza = False
                                    print("El orificio central no posee la circularidad adecuada")

                                # Estudio del centrado del circulo central
                                rospy.loginfo("Segunda función: se comprueba el centrado del circulo central")
                                if (circulo_centrado.main(self.cv_image_color) == "Centrado"):
                                    print("El orificio central está centrado")
                                else:
                                    self.check_pieza = False
                                    print("El orificio central no está centrado")
                                    
                            except Exception as ex:
                                rospy.logerr(ex)
                                break
                            self.frame_tomado = False

                        # Coger el objeto
                        if (self.orientacion_VA == "Orientación x"):
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, trans[0]+0.02, trans[1], 0.25)  ### Punto de acercamiento
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, trans[0]+0.02, trans[1], 0.005)
                            rospy.sleep(0.5)
                            serv_pinza.turn(1)
                            rospy.sleep(0.5)
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, trans[0]+0.02, trans[1], 0.15)  ### Punto de alejamiento
                        elif (self.orientacion_VA == "Orientación y"):
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, trans[0], trans[1]+0.02, 0.25)  ### Punto de acercamiento
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, trans[0], trans[1]+0.02, 0.005)
                            rospy.sleep(0.5)
                            serv_pinza.turn(1)
                            rospy.sleep(0.5)
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, trans[0], trans[1]+0.02, 0.25)  ### Punto de alejamiento

                        # Dejar la pieza en la caja
                        if (self.check_pieza == True):
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, 0, 0.83, 0.17)
                        else:
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, -0.22, 0.85, 0.17)
                        rospy.sleep(0.5)
                        serv_pinza.turn(0)
                        rospy.sleep(0.5)

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
                    # DEBUG
                    # rospy.logdebug("Quaternion del objeto se encuentra en: ", str(rotation))
                    # rospy.logdebug("La transformada a euler del objeto se encuentra en: ", str(tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])))

                    self.resultado = input ("Comprueba en rviz la posición del objeto, ¿seguro que quiere acudir a esa posición? (y or n)")


                    if self.resultado == "y":
                    
                        rospy.sleep(0.5)

                        x ,y, z = IK_moveJ.main_euler_tcp(pi, 0, 0, trans[0], trans[1], 0.005)  ### Para ir perpend al suelo con tcp

                        rospy.sleep(0.5)

                        # Activar pinza de vacío
                        rospy.loginfo("Activando pinza de vacío")
                        if (simulacion == 1):
                            gripper.trigger_gripper(True)
                        else:
                            serv_pinza.turn(1)
                        rospy.sleep(0.5)

                        IK_moveJ.main_euler_tcp(pi, 0, 0, trans[0], trans[1], 0.25)  ### Para ir perpend al suelo con tcp

                        # Dejar la pieza en la caja
                        rospy.loginfo("Dejando objeto")
                        if (self.obj_actual.Class == "esquina"):
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, 0.19, 0.81, 0.17)
                        elif (self.obj_actual.Class == "redonda"):
                            IK_moveL.main_euler_tcp(pi, 0, pi/2, 0.19, 0.81, 0.17)

                        # Depositar objeto
                        rospy.sleep(0.5)
                        rospy.loginfo("Desactivando pinza de vacío")
                        if (simulacion == 1):
                            gripper.trigger_gripper(False)
                        else:
                            serv_pinza.turn(0)
                        rospy.sleep(0.5)
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