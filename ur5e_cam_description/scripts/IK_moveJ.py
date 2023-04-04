#! /usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, cos, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

# Inicializacion, objetos robot y escena, grupos MoveIt
moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('moving_irb120_robot', anonymous=True)
# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")

# Inicializacion, objetos robot y escena, grupos MoveIt
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)



# Inverse Kinematics (IK): mover TCP (brida) a una orientacion dada (Euler) y posicion x, y, z dada. Los angulos se convierten a cuaternios (mensaje de pose)

def move_pose_arm_euler(roll, pitch, yaw, x, y, z):
    pose_goal = geometry_msgs.msg.Pose()
    quat = quaternion_from_euler(roll, pitch, yaw)
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    arm_group.set_pose_target(pose_goal)

    plan = arm_group.go(wait=True)

    arm_group.stop()  # To guarantee no residual movement
    arm_group.clear_pose_targets()

def move_pose_arm_quaternion(quaternion, x, y, z):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    arm_group.set_pose_target(pose_goal)

    plan = arm_group.go(wait=True)

    arm_group.stop()  # To guarantee no residual movement
    arm_group.clear_pose_targets()


def main_euler(roll, pitch, yaw, x, y, z):
    # Los ángulos van en radianes
    move_pose_arm_euler(roll, pitch, yaw, x, y, z)

    rospy.loginfo("Movement finished.")
    # moveit_commander.roscpp_shutdown()


def main_euler_tcp(roll, pitch, yaw, x, y, z):
    # Esta función pasa el punto que tendría el tcp al punto que debe tener el tcp del brazo

    trasl = np.zeros((4,4), np.float)
    trasl[1,1] = trasl[0,0] = trasl[2,2] = trasl[3,3] = 1
    trasl[0,3] = x
    trasl[1,3] = y
    trasl[2,3] = z

    # print(trasl)

    # rotX = np.zeros((4,4), np.float)
    # rotX[0,0] = cos(roll)
    # rotX[0,1] = -sin(roll)
    # rotX[1,0] = sin(roll)
    # rotX[1,1] = cos(roll)
    # rotX[2,2] = rotX[3,3] = 1

    rotX = np.zeros((4,4), np.float)
    rotX[1,1] = cos(roll)
    rotX[1,2] = -sin(roll)
    rotX[2,1] = sin(roll)
    rotX[2,2] = cos(roll)
    rotX[0,0] = rotX[3,3] = 1

    # print(rotX)

    rotY = np.zeros((4,4), np.float)
    rotY[0,0] = cos(pitch)
    rotY[0,2] = sin(pitch)
    rotY[2,0] = -sin(pitch)
    rotY[2,2] = cos(pitch)
    rotY[1,1] = rotY[3,3] = 1

    # print(rotY)

    # rotZ = np.zeros((4,4), np.float)
    # rotZ[1,1] = cos(yaw)
    # rotZ[1,2] = -sin(yaw)
    # rotZ[2,1] = sin(yaw)
    # rotZ[2,2] = cos(yaw)
    # rotZ[0,0] = rotZ[3,3] = 1

    rotZ = np.zeros((4,4), np.float)
    rotZ[0,0] = cos(yaw)
    rotZ[0,1] = -sin(yaw)
    rotZ[1,0] = sin(yaw)
    rotZ[1,1] = cos(yaw)
    rotZ[2,2] = rotZ[3,3] = 1

    # print(rotZ)

    G = np.matmul(rotY, rotX)
    A = np.matmul(rotZ, G)
    B = np.matmul(trasl, A)

    # Datos de la herramienta
    MatHerr = np.zeros((4,4), np.float)
    MatHerr[0,0] = MatHerr[1,1] = MatHerr[2,2] = MatHerr[3,3] = 1
    MatHerr[2,3] = -0.19 ## Longitud provisional

    PosFinal = np.matmul(B, MatHerr)

    # Los ángulos van en grados
    move_pose_arm_euler(roll, pitch, yaw, PosFinal[0,3], PosFinal[1,3], PosFinal[2,3])

    rospy.loginfo("Movement finished.")
    # moveit_commander.roscpp_shutdown()

    return PosFinal[0,3], PosFinal[1,3], PosFinal[2,3]


def main_quaternion(quaternion, x, y, z):
    
    move_pose_arm_quaternion(quaternion, x, y, z)

    rospy.loginfo("Movement finished.")
    # moveit_commander.roscpp_shutdown()


if __name__ == '__main__':

    # Ejemplo de uso de IK (en loop)
    # for i in range(2):
    #     rospy.loginfo("Moving arm to HOME point")
    #     move_pose_arm_euler(0, 90, 0, 0.4, 0, 0.6)
    #     rospy.sleep(1)
    #     rospy.loginfo("Moving arm to point_1")
    #     move_pose_arm_euler(45, 45, 45, 0.5, -0.25, 0.3)
    #     rospy.sleep(1)
    #     rospy.loginfo("Moving arm to point_2")
    #     move_pose_arm_euler(0, -90, 90, 0.5, 0.25, 0.3)
    #     rospy.sleep(1)
    #     rospy.loginfo("Moving arm to point_3")
    #     move_pose_arm_euler(0, 0, 90, 0.2, 0, 0.8)
    #     rospy.sleep(1)
    #     rospy.loginfo("Moving arm to point_4")
    #     move_pose_arm_euler(0, 0, 0, 0, -0.5, 0.4)
    #     rospy.sleep(1)

    move_pose_arm_euler(180, 0, 0, 1, 1, 1)
    rospy.sleep(1)

    rospy.loginfo("Movement finished.")
    moveit_commander.roscpp_shutdown()
