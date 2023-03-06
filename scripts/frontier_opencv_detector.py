#!/usr/bin/env python3


#--------Include modules---------------
import queue
from sympy import arg
import sys

from yaml import Mark
import rclpy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from ros_autonomous_slam_py.getfrontier import getfrontier
from rclpy.exceptions import ROSInterruptException
#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()


def mapCallBack(data):
    global mapData
    mapData=data


    

# Node----------------------------------------------
def node():
		global mapData
		exploration_goal=PointStamped()
		rclpy.init(args=sys.argv)
		nd = rclpy.create_node('detector')
		map_topic= nd.declare_parameter('_map_topic','/map').value
		nd.create_subscription(OccupancyGrid, map_topic, mapCallBack, 10)
		targetspub = nd.create_publisher(PointStamped, '/detected_points', 10)
		pub = nd.create_publisher(Marker, 'shapes', 10)
# wait until map is received, when a map is received, mapData.header.seq will not be < 1
		while len(mapData.data)<1:
			pass
    	   	
		rate = nd.create_rate(50)
		points=Marker()

		#Set the frame ID and timestamp.  See the TF tutorials for information on these.
		points.header.frame_id=mapData.header.frame_id
		points.header.stamp= nd.get_clock().now()
		
		points.ns= "markers"
		points.id = 0

		points.type = Marker.POINTS
		#Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		points.action = Marker.ADD;

		points.pose.orientation.w = 1.0;
		points.scale.x=points.scale.y=0.3;
		points.color.r = 255.0/255.0
		points.color.g = 0.0/255.0
		points.color.b = 0.0/255.0
		points.color.a=1;
		points.lifetime == rclpy.Duration();

#-------------------------------OpenCV frontier detection------------------------------------------
		while not rclpy.is_shutdown():
			frontiers=getfrontier(mapData)
			for i in range(len(frontiers)):
				x=frontiers[i]
				exploration_goal.header.frame_id= mapData.header.frame_id
				exploration_goal.header.stamp=nd.get_clock().now()
				exploration_goal.point.x=x[0]
				exploration_goal.point.y=x[1]
				exploration_goal.point.z=0	


				targetspub.publish(exploration_goal)
				points.points=[exploration_goal.point]
				pub.publish(points)
			rate.sleep()

	  	#rate.sleep()

#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except ROSInterruptException:
        pass
 
 
 
 
