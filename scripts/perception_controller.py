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


velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
pc2_publisher = rospy.Publisher('pointCloud', PointCloud2, queue_size = 1)

vel = Twist ()
evader_marker = Marker ()
line_list_marker = Marker ()
lp = lg.LaserProjection ()


vel.linear.x = 2
vel.linear.y = 0
vel.linear.z = 0
vel.angular.x = 0
vel.angular.y = 0
vel.angular.z = 0


def pubLineMarker(data) :
	global marker_publisher
	global line_list_marker
	global vel
	global velocity_publisher
	line_list_marker.points = []
	rate = rospy.Rate(20)
	ranges = data.ranges
	laser_points = []
	fpts = []
	
	pc2Data = lp.projectLaser(data,3.0)
	pc2_publisher.publish(pc2Data)
	pts = pc2.read_points(pc2Data, skip_nans=True)
	
	for point in pts:
		temp = []
		temp.append(point[0])
		temp.append(point[1])
		temp.append(point[2])
		fpts.append(temp)
	
	th_ptc = 30
	if len(fpts) > th_ptc :
		k = 12
		th_inline = 0.1
		th_itr = 3
		ranPts = []
		zeroDen = 0
		line_list_marker.points = []
		for l in range(th_itr):
			itrPoints = []
			itrInlnr = []
			itrOutlr = []
			inlierPts = []
			for i in range(k):
				inliners = []
				outliers = []
				temp = []
				p1 = random.choice(fpts)
				p2 = random.choice(fpts)
				temp.append(p1)
				temp.append(p2)
				itrPoints.append(temp)
				dist = []
				inlierInd = []
				for ind,pt in enumerate(fpts):
					num = abs(((p2[1]-p1[1])*pt[0]) - ((p2[0]-p1[0])*pt[1]) + (p2[0]*p1[1]) - (p2[1]*p1[0]))
					den = math.sqrt(((p2[1]-p1[1])**2) + ((p2[0]-p1[0])**2))
					if den == 0:
						zeroDen = 1
						break
					d = num/den
					if d < th_inline:
						inliners.append(pt)
						inlierInd.append(ind)
					else:
						outliers.append(pt)
				if zeroDen == 1:
					zeroDen = 0
					continue
				inlierPts.append(inlierInd)
				itrInlnr.append(len(inliners))
				itrOutlr.append(len(outliers))
			if len(itrInlnr) > 0:
				#if max(itrInlnr) > 4:
				index = itrInlnr.index(max(itrInlnr))
				#linePts = itrPoints[index]
				inpts = inlierPts[index]
				linePts = []
				linePts.append(fpts[inpts[0]])
				linePts.append(fpts[inpts[len(inpts)-1]])
				p = Point()
				p.x = linePts[0][0]
				p.y = linePts[0][1]
				p.z = 0.0
				line_list_marker.points.append(p)
				p = Point()
				p.x = linePts[1][0]
				p.y = linePts[1][1]
				p.z = 0.0
				line_list_marker.points.append(p)
				tempinl = inpts[::-1]
				for m in tempinl:
					del fpts[m]
				if len(fpts) < th_ptc:
					break
		marker_publisher.publish(line_list_marker)
	else:
		marker_publisher.publish(line_list_marker)
	rate.sleep()	

'''	
def avoidObstacle (data) :
	global vel
	global velocity_publisher

	wd = 1
	sd = 1
	av = 10
	rmin = min(data.ranges[150:180])
	lmin = min(data.ranges[181:211])
	st = data.ranges[180]	
	rf = data.ranges[90]
	lf = data.ranges[270]
	strgtleft = data.ranges[360]
	offset = 1
	
	if st < wd and lf < sd and rf < sd:
		vel.linear.x = 0
		vel.angular.z = av
		velocity_publisher.publish(vel)
	elif st > wd and lf < sd and rf < sd:
		if rmin < offset or lmin < offset:
			vel.linear.x = 0
			vel.angular.z = av
			velocity_publisher.publish(vel)
		else :
			vel.linear.x = 2
			vel.angular.z = 0
			velocity_publisher.publish(vel)
	elif st < wd or lmin < sd or rmin < sd or rf < sd or lf < sd:
		vel.linear.x = 0
		vel.angular.z = av
		velocity_publisher.publish(vel)
	else:
		vel.linear.x = 2
		vel.angular.z = 0
		velocity_publisher.publish(vel)
'''
	
	


def pubEvaderMarker (data) :
	global evader_marker
	global marker_publisher
	evader_marker.pose.position.x = data.pose.pose.position.x
	evader_marker.pose.position.y = data.pose.pose.position.y
	evader_marker.pose.position.z = data.pose.pose.position.z

	evader_marker.pose.orientation.x = data.pose.pose.orientation.x
	evader_marker.pose.orientation.y = data.pose.pose.orientation.y
	evader_marker.pose.orientation.z = data.pose.pose.orientation.z
	evader_marker.pose.orientation.w = data.pose.pose.orientation.w
	marker_publisher.publish(evader_marker)

def defineEvaderMarker ():
	global evader_marker
	evader_marker.header.frame_id = "odom"
	evader_marker.header.stamp = rospy.Time.now()
	evader_marker.ns = "robot"
	evader_marker.id = 0
	evader_marker.type = Marker.CUBE
	evader_marker.action = Marker.ADD
	evader_marker.scale.x = 0.35
	evader_marker.scale.y = 0.35
	evader_marker.scale.z = 0.25
	evader_marker.color.r = 0.0
	evader_marker.color.g = 1.0
	evader_marker.color.b = 0.0
	evader_marker.color.a = 1.0;
	evader_marker.lifetime = rospy.Duration()

def definePointsAndLinesMarker ():
	global line_list_marker
	line_list_marker.header.frame_id = "base_laser_link"
	line_list_marker.header.stamp = rospy.Time.now()
	line_list_marker.ns = "lines"
	line_list_marker.id = 2
	line_list_marker.type = Marker.LINE_LIST
	line_list_marker.action = Marker.ADD
	line_list_marker.scale.x = 0.1
	line_list_marker.color.r = 1.0
	line_list_marker.color.g = 1.0
	line_list_marker.color.a = 1.0;
	

def evader() :
	global vel
	global velocity_publisher
	rospy.init_node('evader', anonymous=True)
	defineEvaderMarker()
	definePointsAndLinesMarker()
	line_lstnr = rospy.Subscriber('base_scan', LaserScan, pubLineMarker)
	odom_listener = rospy.Subscriber('odom', Odometry, pubEvaderMarker)
	rospy.spin()
	

if __name__ == '__main__':
	try:
	    evader()
	except rospy.ROSInterruptException:
	    pass
		
		
	


