#! /usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Inicializacion, objetos robot y escena, grupos MoveIt
moveit_commander.roscpp_initialize(sys.argv)
# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# Forward Kinematics (FK): movimiento del grupo robot eje a eje (para robots de 6 ejes). Angulos adaptados a grados

def move_joint_arm(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5):
    try:
        joint_goal = arm_group.get_current_joint_values()
        joint_goal[0] = joint_0 * (pi/180)
        joint_goal[1] = joint_1 * (pi/180)
        joint_goal[2] = joint_2 * (pi/180)
        joint_goal[3] = joint_3 * (pi/180)
        joint_goal[4] = joint_4 * (pi/180)
        joint_goal[5] = joint_5 * (pi/180)

        arm_group.go(joint_goal, wait=True)
        arm_group.stop()  # To guarantee no residual movement
    except Exception as e:
        rospy.logerr(e)


def main(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5):
    # print("============ Printing robot state ============")
    # print(robot.get_current_state())
    # print("")

    # rospy.loginfo("Moving arm to selected pose")
    move_joint_arm(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

    # rospy.loginfo("All movements finished. Shutting down")
    rospy.loginfo("Movement finished.")
    # moveit_commander.roscpp_shutdown() # Don't close it here, it wont work!!!!


if __name__ == '__main__':

    # Print estado actual del robot (opcional)
    print("============ Printing robot state ============")
    print(robot.get_current_state())
    print("")

    rospy.init_node('moving_ur5e_robot', anonymous=True)  # I initialitate it here because if is used as import with an other node -> error (only 1 node)

    if len(sys.argv) < 3:
        print("usage: my_node.py arg1 arg2")
        rospy.loginfo("Moving arm to home pose")
        move_joint_arm(0, -90, 90, -90, -90, 180)
    else:
        position = sys.argv[1]
        rospy.loginfo("Moving arm to selected pose")
        move_joint_arm(Number(sys.argv[1]), Number(sys.argv[2]), Number(sys.argv[3]), Number(sys.argv[4]), Number(sys.argv[5]), Number(sys.argv[6]))

    rospy.loginfo("All movements finished. Shutting down")
    moveit_commander.roscpp_shutdown()
