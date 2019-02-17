#!/usr/bin/env python
#license removed for brevity

import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import math 
import random 

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2

roboPos = [-8.0,-2.0]
roboPosOdom = [0,0]
roboOri = [0, 0, 0]
roboOriOdom = [0,0,0]
startPt = []
goalPt = [4.5, 9.0]
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)

vel = Twist ()
lp = lg.LaserProjection ()

GOALSEEK = 1
WALLFOLLOW = 0
GOALFLAG = 0
ATGOAL = 0
GOALROT = 1
linesPts = []
PROGSTART = 1
CONFLICTFLAG = 0
EMPTYSPACECOUNT = 0
a = 0
b = 0
c = 0
vel.linear.x = 0
vel.linear.y = 0
vel.linear.z = 0
vel.angular.x = 0
vel.angular.y = 0
vel.angular.z = 0



def getLineCoeff(p1,p2):
	return (p2[1]-p1[1]),(p1[0]-p2[0]),((p2[0]*p1[1]) - (p1[0]*p2[1]))

def calOnLine(a,b,c,pt):
	ln = (a*pt[0])+(b*pt[1])+c
	return ln

def getBaseInfo(data):
	global roboPos
	global roboOri
	global PROGSTART
	global startPt
	global goalPt
	global a
	global b
	global c
	roboPos[0] = data.pose.pose.position.x
	roboPos[1] = data.pose.pose.position.y
	if PROGSTART == 1:
		startPt.append(roboPos[0])
		startPt.append(roboPos[1])
		PROGSTART = 0
		a,b,c = getLineCoeff(startPt,goalPt)
	qt = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
	roboOri[0],roboOri[1],roboOri[2] = tf.transformations.euler_from_quaternion(qt)

def getOdomInfo(data):
	global roboPosOdom
	global roboOriOdom
	roboPosOdom[0] = data.pose.pose.position.x
	roboPosOdom[1] = data.pose.pose.position.y
	qt = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
	roboOriOdom[0],roboOriOdom[1],roboOriOdom[2] = tf.transformations.euler_from_quaternion(qt)


def baseScanFunc(data):
	global vel
	global velocity_publisher
	global roboPos
	global roboOri
	global startPt
	global goalPt
	global GOALSEEK
	global WALLFOLLOW
	global ATGOAL
	global GOALROT
	global linesPts
	global a
	global b
	global c
	global PROGSTART
	global CONFLICTFLAG
	global EMPTYSPACECOUNT
	if PROGSTART == 1:
		startPt.append(roboPos[0])
		startPt.append(roboPos[1])
		PROGSTART = 0
		a,b,c = getLineCoeff(startPt,goalPt)
	goalThr = 0.2
	if ATGOAL == 0:
		d = calGoalDistance(goalPt,roboPos)
		linang = math.atan2(goalPt[1]-roboPos[1],goalPt[0]-roboPos[0])
		st = calOnLine(a,b,c,roboPos)
		
		if d < goalThr:
			vel.linear.x = 0
			vel.angular.z = 0	
			ATGOAL = 1
			GOALSEEK = 0
			WALLFOLLOW = 0
			velocity_publisher.publish(vel)
		else:
			if GOALSEEK == 1:
				if checkForGoalReach(goalThr) == False:
					print("goal follow")
					vel.angular.z = calGoalSeekRot(linang,data,0,roboOri[2])
					if vel.angular.z == 0 :
						vel.linear.x = 0.5
					if checkObstacle(data) == True:
						if st < 0.0 and st > -3:
							CONFLICTFLAG = 1
						print("inside goal seek ,dist")
						GOALSEEK = 0
						WALLFOLLOW = 1
						vel.linear.x = 0
						vel.angular.z = 0
					else:
						CONFLICTFLAG = 0
					velocity_publisher.publish(vel)
					checkForGoalReach(goalThr)
			if WALLFOLLOW == 1:
				if checkForGoalReach(goalThr) == False:
					print("wall follow")
					vel.angular.z = calWFRot(data)
					if vel.angular.z < 0.02:
						vel.linear.x = 0.5
					velocity_publisher.publish(vel)
					if min(data.ranges) < 1.5:
						EMPTYSPACECOUNT = 0
					if st >= 0.0 or st <= -3:
						CONFLICTFLAG = 0
					if st < 0.0 and st > -3 and CONFLICTFLAG == 0:
						GOALSEEK = 1
						WALLFOLLOW = 0
						vel.linear.x = 0
						vel.angular.z = calGoalSeekRot(linang,data,1,roboOri[2])
						velocity_publisher.publish(vel)
						print("on the goal seek line, set goalseek = 1")
					elif min(data.ranges) > 1.5 :
						EMPTYSPACECOUNT+=1
						if EMPTYSPACECOUNT > 30:
							EMPTYSPACECOUNT = 0
							GOALSEEK = 1
							WALLFOLLOW = 0
							vel.linear.x = 0
							vel.angular.z = calGoalSeekRot(linang,data,1,roboOri[2])
							velocity_publisher.publish(vel)
					checkForGoalReach(goalThr)

def checkOpenArea(data):
	if min(data.ranges) == 3:
		return True
	else:
		return False

def checkForGoalReach(thr):
	global vel
	global velocity_publisher
	global roboPos
	global goalPt
	global GOALSEEK
	global WALLFOLLOW
	global ATGOAL
	if calGoalDistance(goalPt,roboPos) < thr:
		vel.linear.x = 0
		vel.angular.z = 0	
		ATGOAL = 1
		GOALSEEK = 0
		WALLFOLLOW = 0
		velocity_publisher.publish(vel)
		return True
	else:
		return False

def calWFRot(data):
	wd = 0.8
	sd = 0.8
	av = 10
	z = 0
	rmin = min(data.ranges[90:180])
	lmin = min(data.ranges[181:271])
	st = data.ranges[180]	
	rf = min(data.ranges[0:90])
	lf = min(data.ranges[271:361])

	if lmin < 0.8 or st < 1:
		z = -2
	else:
		if lf > 0.8:
			z = 5
	return 	z

def checkObstacle(data):
	wd = 0.8
	sd = 0.8
	av = 10
	rmin = min(data.ranges[150:180])
	lmin = min(data.ranges[181:211])
	st = data.ranges[180]	
	rf = min(data.ranges[0:90])
	lf = min(data.ranges[271:361])
	offset = 1
	if st < wd or lmin < sd or rmin < sd:
		return True
	else:
		return False


def calGoalDistance(gpt,rpt):
	d = math.sqrt(((gpt[1]-rpt[1])**2) + ((gpt[0]-rpt[0])**2))
	return d

def calGoalSeekRot(linang,data,flag,roboOri):
	angle = linang - roboOri - (math.pi/20)	
	rf = min(data.ranges[0:90])
	lf = min(data.ranges[271:361])
	if abs(angle) < (math.pi/20):
		return 0
	else:
		if flag == 1:
			if lf < 0.8 and angle > 0:
				return 	angle * -10
			elif rf < 0.8 and angle < 0:
 				return 	angle * -10
			else:
       				return angle * 10
		else:			
			return angle * 10
		


def evader() :
	rospy.init_node('evader', anonymous=True)
	#line_pos_lstnr = rospy.Subscriber('visualization_marker', Marker, getLinePos)
	base_lstnr = rospy.Subscriber('base_pose_ground_truth', Odometry, getBaseInfo)
	odom_lstnr = rospy.Subscriber('odom', Odometry, getOdomInfo)
	scan = rospy.Subscriber('base_scan', LaserScan, baseScanFunc)
	rospy.spin()

if __name__ == '__main__':
	try:
	    evader()
	except rospy.ROSInterruptException:
	    pass
		
		
	


