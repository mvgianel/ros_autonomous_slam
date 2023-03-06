#!/usr/bin/env python3

#--------Include modules---------------
from copy import copy
import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from ros_autonomous_slam.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from ros_autonomous_slam_py.functions import robot,informationGain,discount
from numpy.linalg import norm

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]
def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------

def node():
	global frontiers,mapData,global1,global2,global3,globalmaps
	rclpy.init()
	node = rclpy.create_node('assigner')
	
	# fetching all parameters
	map_topic= node.declare_parameter('_map_topic','map').value
	info_radius= node.declare_parameter('_info_radius',1.0).value			#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=node.declare_parameter('_info_multiplier',3.0).value		
	hysteresis_radius=node.declare_parameter('_hysteresis_radius',3.0).value			#at least as much as the laser scanner range
	hysteresis_gain=node.declare_parameter('_hysteresis_gain',2.0).value				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= node.declare_parameter('_frontiers_topic','/filtered_points').value	
	n_robots = node.declare_parameter('_n_robots',1).value
	namespace = node.declare_parameter('_namespace','husky').value
	namespace_init_count = node.declare_parameter('_namespace_init_count',1).value
	delay_after_assignement=node.declare_parameter('_delay_after_assignement',0.5).value
	rateHz = node.declare_parameter('_rate',100).value
	
	rate = node.create_rate(rateHz)
#-------------------------------------------
	depth = 10
	node.create_subscription(OccupancyGrid, map_topic, mapCallBack, depth)
	node.create_subscription(PointArray, frontiers_topic, callBack, depth)
#---------------------------------------------------------------------------------------------------------------
		
# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)	
#wait if map is not received yet
	while (len(mapData.data)<1):
		pass	
	robots=[]
	if len(namespace)>0:
		for i in range(0,n_robots):
			robots.append(robot(namespace+str(i+namespace_init_count)))
	elif len(namespace)==0:
			robots.append(robot(namespace))	
	for i in range(0,n_robots):
		robots[i].sendGoal(robots[i].getPosition())
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while rclpy.ok():	
		centroids=copy(frontiers)

#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------			
#get number of available/busy robots
		na=[] #available robots
		nb=[] #busy robots
		for i in range(0,n_robots):
			if (robots[i].getState()==1):
				nb.append(i)
			else:
				na.append(i)	
		node.get_logger().info("available robots: "+str(na))	
#------------------------------------------------------------------------- 
#get dicount and update informationGain
		for i in nb+na:
			infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------            
		revenue_record=[]
		centroid_record=[]
		id_record=[]
		
		for ir in na:
			for ip in range(0,len(centroids)):
				cost=norm(robots[ir].getPosition()-centroids[ip])		
				threshold=1
				information_gain=infoGain[ip]
				if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):

					information_gain*=hysteresis_gain
				revenue=information_gain*info_multiplier-cost
				revenue_record.append(revenue)
				centroid_record.append(centroids[ip])
				id_record.append(ir)
		
		if len(na)<1:
			revenue_record=[]
			centroid_record=[]
			id_record=[]
			for ir in nb:
				for ip in range(0,len(centroids)):
					cost=norm(robots[ir].getPosition()-centroids[ip])		
					threshold=1
					information_gain=infoGain[ip]
					if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
						information_gain*=hysteresis_gain
				
					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain

					revenue=information_gain*info_multiplier-cost
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)
		
		node.get_logger().info("revenue record: "+str(revenue_record))	
		node.get_logger().info("centroid record: "+str(centroid_record))	
		node.get_logger().info("robot IDs record: "+str(id_record))	
		
#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
			node.get_logger().info(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))	
			rate.sleep(delay_after_assignement)
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except KeyboardInterrupt:
        pass
 
