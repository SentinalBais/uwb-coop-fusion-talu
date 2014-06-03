#!/usr/bin/env python  
import rospy
import tf
import math
 
'''
Do not run this node when publishing frames using ROSbags, because timestamps will not match 
'''

DEG2RAD = math.pi/180.0     		   		    #constant convert degrees to radians

def broadcast():
	rospy.init_node('rviz_flip')
	br = tf.TransformBroadcaster()  #transform camera to body frame of robot 
	br2 = tf.TransformBroadcaster()
	rate = rospy.Rate(10) # 1 Hz publish rate 
	while not rospy.is_shutdown(): # publish at 10hz	
		br.sendTransform((1,0,0), 
		tf.transformations.quaternion_from_euler(0.0*DEG2RAD, 0.0*DEG2RAD, 20.0*DEG2RAD), #ax, ay, az 
		rospy.Time.now(),
		"/f1", # end 
		"/map") # refference coordinate system - frame where angles/translations are being measured from

		br.sendTransform((2,0,0), 
		tf.transformations.quaternion_from_euler(0.0*DEG2RAD, 20.0*DEG2RAD, 20.0*DEG2RAD), #ax, ay, az 
		rospy.Time.now(),
		"/f2", # end 
		"/f1") # refference coordinate system - frame where angles/translations are being measured from

		br.sendTransform((3,0,0), 
		tf.transformations.quaternion_from_euler(0.0*DEG2RAD, 20.0*DEG2RAD, 20.0*DEG2RAD), #ax, ay, az 
		rospy.Time.now(),
		"/f3", # end 
		"/f2") # refference coordinate system - frame where angles/translations are being measured from

		br2.sendTransform((0,0,3), 
		tf.transformations.quaternion_from_euler(20.0*DEG2RAD, 20.0*DEG2RAD, 20.0*DEG2RAD), #ax, ay, az 
		rospy.Time.now(),
		"/f4", # end
		"/map") # refference coordinate system - frame where angles/translations are being measured from
		rate.sleep()


if __name__ == '__main__':
	broadcast()


