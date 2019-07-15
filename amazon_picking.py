# -*- coding: utf-8 -*-
#!/usr/bin/env python

"""
    moveit_cartesian_demo.py - Version 0.1 2014-01-14
    
    Plan and execute a Cartesian path for the end-effector through a number of waypoints
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""
# modified by liguobin
import rospy, sys
import moveit_commander
import agile_grasp2.srv as ag
import tf
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from objrecog_msgs.srv import object_pose
from gpd.srv import npy
from gpd.msg import GraspConfigList
import copy
import numpy as np
import serial
serialPort="/dev/ttyUSB0"   #串口
baudRate=9600       #波特率

ser=serial.Serial(serialPort,baudRate,timeout=0.5)
print "参数设置：串口=%s ，波特率=%d"%(serialPort,baudRate)
    
#str1 = raw_input("请输入要发送的数据（非中文）并同时接收数据: ")
#print str1
ser.write(('0\n').encode())
rospy.sleep(2)
ser.write(('0\n').encode())
rospy.sleep(2)
print '.'
print(ser.readline())#可以接收中文


getkey=0
RIGHT_ARM = 1
LEFT_ARM = 1
def current_pose():
        print "pointa"
        listener = tf.TransformListener()
	rateinner = rospy.Rate(40.0)
    	while not rospy.is_shutdown():
          try:
             (transA,rotA) = listener.lookupTransform("/base_link", "/ee_link", rospy.Time(0))

	     maxtrix=tf.transformations.quaternion_matrix(rotA)

            # print 'currentmaxtrix',maxtrix
             break
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
             continue
 
          rateinner.sleep()
        print "out of the xunhuan"


        first_pose = Pose()
 
        first_pose.position.x  = transA[0]
        first_pose.position.y  = transA[1]
        first_pose.position.z  = transA[2]
        first_pose.orientation.x=rotA[0]
        first_pose.orientation.y=rotA[1]
        first_pose.orientation.z=rotA[2]
        first_pose.orientation.w=rotA[3]

 
        return first_pose



def current_cameralinkpose():
        print "pointa"
        listener = tf.TransformListener()
	rateinner = rospy.Rate(40.0)
    	while not rospy.is_shutdown():
          try:
             (transA,rotA) = listener.lookupTransform("/base_link", "/camera_joint", rospy.Time(0))
             print transA
             break
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
             continue
 
          rateinner.sleep()
        print "out of the xunhuan"


        first_pose = Pose()
 
        first_pose.position.x  = transA[0]
        first_pose.position.y  = transA[1]
        first_pose.position.z  = transA[2]
        first_pose.orientation.x=rotA[0]
        first_pose.orientation.y=rotA[1]
        first_pose.orientation.z=rotA[2]
        first_pose.orientation.w=rotA[3]

 
        return first_pose





def catisian_moving(pose,right_arm):

 
 



        # Initialize the waypoints list
        waypoints = pose
        print waypoints

 
 
	fraction = 0.0
	maxtries = 100
	attempts = 0

	# Set the internal state to the current state
	right_arm.set_start_state_to_current_state()

	# Plan the Cartesian path connecting the waypoints
	while fraction < 1.0 and attempts < maxtries:
	 (plan, fraction) = right_arm.compute_cartesian_path (
		                waypoints,   # waypoint poses
		                0.01,        # eef_step
		                0.0,         # jump_threshold
		                True)        # avoid_collisions

	# Increment the number of attempts 
	attempts += 1

	# Print out a progress message
	if attempts % 10 == 0:
	    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
		 
	# If we have a complete plan, execute the trajectory
	if fraction == 1.0:
		rospy.loginfo("Path computed successfully. Moving the arm.")
		joints = tuple(right_arm.get_current_joint_values()) 
		rospy.sleep(5)
		#rospy.loginfo(plan)
		right_arm.execute(plan)
			    
		rospy.loginfo("Path execution complete.")
	else:
		rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
	




def graspcallback(msg):
    global getkey
    global grasps
    grasps = msg.grasps[0]
    print 'grasps:',grasps.approach,grasps.sample

    print 'grasps,detail:',grasps.approach.x,grasps.approach.y,grasps.approach.z,grasps.sample.x,grasps.sample.y,grasps.sample.z
    getkey=1

def transformgrasp(currentcamera,grasps,distanceA,distanceB):


        trans1_mat = tf.transformations.translation_matrix([currentcamera.position.x,currentcamera.position.y,currentcamera.position.z])
        rot1_mat   = tf.transformations.quaternion_matrix([currentcamera.orientation.x,currentcamera.orientation.y,currentcamera.orientation.z,currentcamera.orientation.w])
        mat1 = np.dot(trans1_mat, rot1_mat)
        print 'currentcamera', mat1,[currentcamera.orientation.x,currentcamera.orientation.y,currentcamera.orientation.z,currentcamera.orientation.w],[currentcamera.position.x,currentcamera.position.y,currentcamera.position.z]


	origraspposition=np.array([grasps.sample.x,grasps.sample.y,grasps.sample.z])
	finalgraspposition=origraspposition-distanceA*np.array([grasps.approach.x,grasps.approach.y,grasps.approach.z])

	print ('origrasppositionnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn',origraspposition)

	trans2_mat = tf.transformations.translation_matrix(finalgraspposition)
	rot2_mat    = np.array([[grasps.approach.x,grasps.axis.x,-grasps.binormal.x,0],[grasps.approach.y,grasps.axis.y,-grasps.binormal.y,0],[grasps.approach.z,grasps.axis.z,-grasps.binormal.z,0],[0,0,0,1]])
 
	#print ('rot2_mat',rot2_mat)
#tf.transformations.quaternion_matrix(rot2)
	mat2 = np.dot(trans2_mat, rot2_mat)




	mat3 = np.dot(mat1, mat2)
	trans3 = tf.transformations.translation_from_matrix(mat3)
	rot3 = tf.transformations.quaternion_from_matrix(mat3)

        end_pose = Pose()
        end_pose.position.x  = trans3[0]
        end_pose.position.y  = trans3[1]
        end_pose.position.z  = trans3[2]
        end_pose.orientation.x=rot3[0]
        end_pose.orientation.y=rot3[1]
        end_pose.orientation.z=rot3[2]
        end_pose.orientation.w=rot3[3]












	origraspposition2=np.array([grasps.sample.x,grasps.sample.y,grasps.sample.z])
	finalgraspposition2=origraspposition2-distanceB*np.array([grasps.approach.x,grasps.approach.y,grasps.approach.z])



	trans2_mat2 = tf.transformations.translation_matrix(finalgraspposition2)
	rot2_mat2    = np.array([[grasps.approach.x,grasps.axis.x,-grasps.binormal.x,0],[grasps.approach.y,grasps.axis.y,-grasps.binormal.y,0],[grasps.approach.z,grasps.axis.z,-grasps.binormal.z,0],[0,0,0,1]])
 
	print ('rot2_mat',rot2_mat2)
#tf.transformations.quaternion_matrix(rot2)
	mat2_2 = np.dot(trans2_mat2, rot2_mat2)




	mat3_2 = np.dot(mat1, mat2_2)
	trans3_2 = tf.transformations.translation_from_matrix(mat3_2)
	rot3_2 = tf.transformations.quaternion_from_matrix(mat3_2)

        end_pose_2 = Pose()
        end_pose_2.position.x  = trans3_2[0]
        end_pose_2.position.y  = trans3_2[1]
        end_pose_2.position.z  = trans3_2[2]
        end_pose_2.orientation.x=rot3_2[0]
        end_pose_2.orientation.y=rot3_2[1]
        end_pose_2.orientation.z=rot3_2[2]
        end_pose_2.orientation.w=rot3_2[3]











        return  end_pose,end_pose_2



def graspobject():

                


                 
        # Connect to the right_arm move group
        right_arm = MoveGroupCommander('manipulator')
	right_arm.set_planner_id('RRTConnectkConfigDefault')   

    
        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Set the right arm reference frame
        right_arm.set_pose_reference_frame('base_link')
       
               
        # Allow some leeway in position(meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.1)
      
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
	#right_arm.set_end_effector_link(end_effector_link)
    

	#start_pose = Pose()
        #start_pose.position.x = 0.587  
        #start_pose.position.y = 0.034 
        #start_pose.position.z =  -0.010
        #start_pose.orientation.x =0.508 
        #start_pose.orientation.y =  0.508  
        #start_pose.orientation.z =  -0.491 
        #start_pose.orientation.w =  0.491
        #right_arm.set_pose_target(start_pose)    
        #traj = right_arm.plan()
	#rospy.sleep(2)
        #right_arm.execute(traj)
        #rospy.sleep(2)

 

 
 	global getkey
        print '回到初始位置结束'

	startclient = rospy.ServiceProxy('/detect_grasps/actionstart', npy)  
	startclient(0)
        print 'call start over'

  	sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, graspcallback)    
	rate = rospy.Rate(200)
	while not rospy.is_shutdown(): 

                if getkey==1:
                   break

 	        rate.sleep()
	getkey=0

	global grasps
	print '检测到抓取位置，开始执行'



        global ser
        ser.write(('0\n').encode())
        rospy.sleep(2)


	print "-start movingA------------------------------------"
	pose = []
	currentcamera=current_cameralinkpose()
        grasppose1,grasppose2= transformgrasp(currentcamera,grasps,0.24,0.15)
	start_pose = current_pose()
        print 'start_pose',start_pose
	pose.append(start_pose)
        pose.append(grasppose1)
        pose.append(grasppose2)
	#end_pose = copy.deepcopy(start_pose)
       # end_pose.position.x = grasps.approach.x 
      #  end_pose.position.y = grasps.approach.y 
      #  end_pose.position.z =  grasps.approach.z 

 	catisian_moving(pose,right_arm)#迪卡尔插值到目标姿态
	print "-start movingB------------------------------------"
	print '抓' 
        ser.write(('1\n').encode())
        rospy.sleep(2)

##############################################################
 


	group_variable_values = right_arm.get_current_joint_values()
        print "============ Joint values: ", group_variable_values 
	group_variable_values[0] =1.5740725994110107 #机器人的观测姿态定义在这里
	group_variable_values[1] =  -1.5415833632098597
	group_variable_values[2] =   -1.2562201658831995
	group_variable_values[3] = -1.8575199286090296
	group_variable_values[4] = 1.572489857673645
	group_variable_values[5] =   1.5713907480239868
	right_arm.set_joint_value_target(group_variable_values)

	plan2 = right_arm.plan()
        
	print "============ Waiting while RVIZ displays plan2..."
	rospy.sleep(1)
 
        right_arm.execute(plan2)#先移到观测姿态  













	group_variable_values = right_arm.get_current_joint_values()
        print "============ Joint values: ", group_variable_values 
	group_variable_values[0] =2.6649599075317383#机器人的观测姿态定义在这里
	group_variable_values[1] =  -1.493981663380758
	group_variable_values[2] =  -1.7679532209979456
	group_variable_values[3] =-1.3932693640338343
	group_variable_values[4] =  1.5719022750854492
	group_variable_values[5] =    1.5657243728637695
	right_arm.set_joint_value_target(group_variable_values)

	plan2 = right_arm.plan()
        
	print "============ Waiting while RVIZ displays plan2..."
	rospy.sleep(2)
 
        right_arm.execute(plan2)#先移到观测姿态  
	print "-start movingB------------------------------------"
	print '笛卡尔插值到放置位置完成' 
        ser.write(('0\n').encode())
        rospy.sleep(2)

       # pose = []
        #posecurrent=current_pose()#获取现有的机器人末端姿态
        
#        pose.append(posecurrent)

	#start_pose = Pose()#似乎这里不能接受相对的移动
        #start_pose= objectpose#获取目标末端姿态
 #       posecurrent.position.z =posecurrent.position.z -0.1



  #      pose.append(posecurrent)
       # print start_pose
        print "-------------------------------------"
       # print pose
        print "-------------------------------------"
   #     catisian_moving(pose,right_arm)#迪卡尔插值到目标姿态

	#回到初始位置
	group_variable_values = right_arm.get_current_joint_values()
        print "============ Joint values: ", group_variable_values 
	group_variable_values[0] =1.5740725994110107 #机器人的观测姿态定义在这里
	group_variable_values[1] =  -1.5415833632098597
	group_variable_values[2] =   -1.2562201658831995
	group_variable_values[3] = -1.8575199286090296
	group_variable_values[4] = 1.572489857673645
	group_variable_values[5] =   1.5713907480239868
	right_arm.set_joint_value_target(group_variable_values)

	plan2 = right_arm.plan()
        
	print "============ Waiting while RVIZ displays plan2..."
	rospy.sleep(1)
 
        right_arm.execute(plan2)#先移到观测姿态  
	rospy.sleep(3)
  
        print '回到初始位置结束'

	startclient2 = rospy.ServiceProxy('/detect_grasps/actionover', npy)  
	startclient2(0)
        print 'call start over'
 


class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        print 'starting'
        moveit_commander.roscpp_initialize(sys.argv)
 



                 
        # Connect to the right_arm move group
        right_arm = MoveGroupCommander('manipulator')
	right_arm.set_planner_id('RRTConnectkConfigDefault')   

    
        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Set the right arm reference frame
        right_arm.set_pose_reference_frame('base_link')
       
               
        # Allow some leeway in position(meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.1)
      
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
	#right_arm.set_end_effector_link(end_effector_link)





        # Get the current pose so we can add it as a waypoint
	group_variable_values = right_arm.get_current_joint_values()
        print "============ Joint values: ", group_variable_values 
	group_variable_values[0] =1.5740725994110107 #机器人的观测姿态定义在这里
	group_variable_values[1] =  -1.5415833632098597
	group_variable_values[2] =   -1.2562201658831995
	group_variable_values[3] = -1.8575199286090296
	group_variable_values[4] = 1.572489857673645
	group_variable_values[5] =   1.5713907480239868
	right_arm.set_joint_value_target(group_variable_values)

	plan2 = right_arm.plan()
        
	print "============ Waiting while RVIZ displays plan2..."
	rospy.sleep(2)
 
        right_arm.execute(plan2)#先移到观测姿态  






	try:
		 
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
		    print '.'
		    graspobject( )
		    rate.sleep()

	 

	except rospy.ROSInterruptException:
		pass
        # Initialize the ROS node
       # rospy.init_node('moveit_demo', anonymous=True)
        
    #    s = rospy.Service('startmotion', ag.npy, single_to_double)
 
 
 

	#for i in range(5):
	#try:
	#    objectpose = val(i)
	 #   print objectpose
	#except rospy.ServiceException, e:
	   # print e
	#rospy.sleep(2)
 
	






#	val = rospy.ServiceProxy('action', ag.npy)

        #try:
        #    resp1 = val(1)
        #    print resp1.success, resp1.message
        #except rospy.ServiceException, e:
         #   print e



 

if __name__ == "__main__":

    print('send open')#可以接收中文
    rospy.init_node('mainactions')
    MoveItDemo()


