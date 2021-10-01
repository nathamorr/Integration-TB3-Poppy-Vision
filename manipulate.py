#!/usr/bin/env python3
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from poppy_ros_control.recorder import Player
import rospy

rospy.init_node('ros4pro_custom_node')
rate = rospy.Rate(1)
commander = MoveGroupCommander("arm_and_finger", wait_for_servers=20)
player = Player()

#Initialisation des params
rospy.set_param("/label", -1)
rospy.set_param("/targetLabel", -1)
rospy.set_param("/robotReady", True)

#Initialisation image
rospy.set_param("/takeImage", True)
print("manipulate | /takeImage -> True")

#Position du cube (a, b, c, d)
positionCube = 1

while not rospy.is_shutdown():
    #rospy.loginfo("Hello world from our new node!")

    # Reset de la position du robot
    print("Reset position")
    commander.set_joint_value_target([0, 0, 0, 0, 0, 0])
    commander.go()
    
    print(rospy.get_param("/label"))

    if(rospy.get_param("/label") != -1):
        print(rospy.get_param("/label"))
        
        if(positionCube == 1):

            print("Grab du cube n°1")
            my_motion = player.load("mouvement_a")

            # Go to the start position before replaying the motion
            commander.set_joint_value_target(my_motion.joint_trajectory.points[0].positions)
            commander.go()

            # Replay the exact same motion
            commander.execute(my_motion)
            rospy.set_param("/targetLabel", rospy.get_param("/label"))
            
            positionCube += 1

        elif(positionCube == 2):
            print("Grab du cube n°2")
            my_motion = player.load("mouvement_b")

            # Go to the start position before replaying the motion
            commander.set_joint_value_target(my_motion.joint_trajectory.points[0].positions)
            commander.go()

            # Replay the exact same motion
            commander.execute(my_motion)
            rospy.set_param("/targetLabel", rospy.get_param("/label"))
            
            positionCube += 1

        elif(positionCube == 3):
            print("Grab du cube n°3")
            my_motion = player.load("mouvement_c")

            # Go to the start position before replaying the motion
            commander.set_joint_value_target(my_motion.joint_trajectory.points[0].positions)
            commander.go()

            # Replay the exact same motion
            commander.execute(my_motion)
            rospy.set_param("/targetLabel", rospy.get_param("/label"))
            
            positionCube += 1

        elif(positionCube == 4):
            print("Grab du cube n°4")
            my_motion = player.load("mouvement_d")

            # Go to the start position before replaying the motion
            commander.set_joint_value_target(my_motion.joint_trajectory.points[0].positions)
            commander.go()

            # Replay the exact same motion
            commander.execute(my_motion)
            rospy.set_param("/targetLabel", rospy.get_param("/label"))

            print("manipulate | shutdown")
            rospy.signal_shutdown("Shutdown")

    else :
        print("manipulate | Attente d'un label")

    rospy.set_param("/label", -1)
        
    rate.sleep()
