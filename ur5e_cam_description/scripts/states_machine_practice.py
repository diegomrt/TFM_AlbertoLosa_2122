#!/usr/bin/env python3

import rospy
import roslaunch
import smach
from darknet_ros_msgs.msg import BoundingBoxes
import threading


# define state Camera
class Camera(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['Success','Failed'],
                                    input_keys=['launch_camera'],
                                    output_keys=['launch_camera'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Camera')
        try:
            rospy.loginfo("Archivo launch lanzado.")
            # Define object launch
            userdata.launch_camera = None
            ruta_camera = "/opt/ros/noetic/share/realsense2_camera/launch/rs_camera.launch"
            # Ruta ABSOLUTA del archivo launch
            uuid_camera = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid_camera)
            userdata.launch_camera = roslaunch.parent.ROSLaunchParent(uuid_camera, [ruta_camera])
            userdata.launch_camera.start()
            rospy.sleep(15)
            return 'Success'
        except Exception as ex:
            rospy.logerr(ex)
            return 'Failed'


# define state Moving
class Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['Success','Failed'],
                                    input_keys=['launch_camera_2'])
        self.data = None
        self.get_data_subscriber = None
        self.bounding_box = None
        self.get_data_event = threading.Event()

    def get_data_callback(self, data):
        self.data = data
        for self.i in (self.data.bounding_boxes):
            self.bounding_box = self.i
            print(self.bounding_box.Class)
        self.get_data_subscriber.unregister()
        self.get_data_subscriber = None
        self.get_data_event.set()

    def execute(self, userdata):
        rospy.loginfo('Executing state Moving')

        try:
            rospy.loginfo("Ejecutando Yolo")
            ruta_yolo = "/home/alberto/tfm_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch"
            uuid_yolo = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid_yolo)
            launch_yolo = roslaunch.parent.ROSLaunchParent(uuid_yolo, [ruta_yolo])
            launch_yolo.start()

            rospy.sleep(20)

            self.get_data_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.get_data_callback, queue_size=1) #Seguir aquí (hacer función feedback)

            while True:
                self.get_data_event.wait(0.1)
                if self.get_data_event.is_set():
                    self.get_data_event.clear()
                    break
                if rospy.is_shutdown():
                    return
            
            rospy.logwarn('Closing both launch processes')

            rospy.loginfo(userdata.launch_camera_2)
            userdata.launch_camera_2.shutdown()
            launch_yolo.shutdown()

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
    sm.userdata.sm_launch_camera = None

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CAMERA', Camera(), 
                                transitions={'Success':'MOVING', 
                                            'Failed':'CAMERA'},
                                remapping={'launch_camera':'sm_launch_camera'})
        smach.StateMachine.add('MOVING', Moving(), 
                                transitions={'Success':'CAMERA',
                                            'Failed':'MOVING'},
                                remapping={'launch_camera_2':'sm_launch_camera'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()