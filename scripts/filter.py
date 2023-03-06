#!/usr/bin/env python3

# --------Include modules---------------
from copy import copy
from operator import length_hint
import sys
import rclpy
from rosidl_generator_py import import_type_support
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import tf2_ros
from numpy import array, vstack, delete
from ros_autonomous_slam_py.functions import gridValue, informationGain
from sklearn.cluster import MeanShift
from ros_autonomous_slam.msg import PointArray
from rclpy.duration import Duration
from rclpy.time import Time

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
globalmaps = []


def callBack(data, args):
    global frontiers, min_distance
    transformedPoint = args[0].transformPoint(args[1], data)
    x = [array([transformedPoint.point.x, transformedPoint.point.y])]
    if len(frontiers) > 0:
        frontiers = vstack((frontiers, x))
    else:
        frontiers = x


def mapCallBack(data):
    global mapData
    mapData = data


def globalMap(data):
    global global1, globalmaps, litraIndx, namespace_init_count, n_robots
    global1 = data
    if n_robots > 1:
        indx = int(data._connection_header['topic']
                   [litraIndx])-namespace_init_count
    elif n_robots == 1:
        indx = 0
    globalmaps[indx] = data

# Node----------------------------------------------


def node():
    global frontiers, mapData, global1, global2, global3, globalmaps, litraIndx, n_robots, namespace_init_count
    rclpy.init()

    node = rclpy.create_node('filter')
    # fetching all parameters
    map_topic = node.declare_parameter('_map_topic', 'map').value
    threshold = node.declare_parameter('_costmap_clearing_threshold', 70).value
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_radius = node.declare_parameter('_info_radius', 1.0).value
    goals_topic = node.declare_parameter('_goals_topic', '/detected_points').value
    n_robots = node.declare_parameter('_n_robots', 1).value
    namespace = node.declare_parameter('_namespace', 'husky').value
    namespace_init_count = node.declare_parameter('_namespace_init_count', 1).value
    rateHz = node.declare_parameter('_rate', 100).value
    global_costmap_topic = node.declare_parameter(
        'global_costmap_topic', '/move_base/global_costmap/costmap').value
    robot_frame = node.declare_parameter('_robot_frame', 'base_link').value

    litraIndx = len(namespace)
    rate = node.create_rate(rateHz)
# -------------------------------------------
    length = 10
    node.create_subscription(OccupancyGrid, map_topic, mapCallBack, length)


# ---------------------------------------------------------------------------------------------------------------

    for i in range(0, n_robots):
        globalmaps.append(OccupancyGrid())

    if len(namespace) > 0:
        for i in range(0, n_robots):
            node.create_subscription(OccupancyGrid, namespace+str(i+namespace_init_count) +
                             global_costmap_topic, globalMap, length)
    elif len(namespace) == 0:
        node.create_subscription(OccupancyGrid, global_costmap_topic, globalMap, length)
# wait if map is not received yet
    while (len(mapData.data) < 1):
        node.get_logger().info('Waiting for the map')
        rate = node.create_rate(0.1)
        pass
# wait if any of robots' global costmap map is not received yet
    for i in range(0, n_robots):
        while (len(globalmaps[i].data) < 1):
            node.get_logger().info('Waiting for the global costmap')
            rate.sleep(0.1)
            pass

    global_frame = "/"+mapData.header.frame_id

    tf2_buffer = tf2_ros.Buffer()
    tf2_rosLisn = tf2_ros.TransformListener(tf2_buffer)
    if len(namespace) > 0:
        for i in range(0, n_robots):

            tf2_buffer.wait_for_transform_async(global_frame[1:], namespace+str(
                i+namespace_init_count)+'/'+robot_frame, rclpy.Time(0))
    elif len(namespace) == 0:
        tf2_buffer.wait_for_transform_async(global_frame[1:], '/'+robot_frame, rclpy.Time(0))

    node.create_subscription(PointStamped, goals_topic, callback=callBack,
                     callback_args=[tf2_rosLisn, global_frame[1:]])
    pub = node.create_publisher(Marker, 'frontiers', queue_size=10)
    pub2 = node.create_publisher(Marker, 'centroids', queue_size=10)
    filterpub = node.create_publisher(PointArray, 'filtered_points', queue_size=10)

    node.get_logger().info("the map and global costmaps are received")

    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass

    points = Marker()
    points_clust = Marker()
# Set the frame ID and timestamp.  See the tf2_ros tutorials for information on these.
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = node.get_clock().now()

    points.ns = "markers2"
    points.id = 0

    points.type = Marker.POINTS

# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0

    points.scale.x = 0.2
    points.scale.y = 0.2

    points.color.r = 255.0/255.0
    points.color.g = 255.0/255.0
    points.color.b = 0.0/255.0

    points.color.a = 1
    points.lifetime = Duration()

    p = Point()

    p.z = 0

    pp = []
    pl = []

    points_clust.header.frame_id = mapData.header.frame_id
    points_clust.header.stamp = node.get_clock().now()

    points_clust.ns = "markers3"
    points_clust.id = 4

    points_clust.type = Marker.POINTS

# Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points_clust.action = Marker.ADD

    points_clust.pose.orientation.w = 1.0

    points_clust.scale.x = 0.2
    points_clust.scale.y = 0.2
    points_clust.color.r = 0.0/255.0
    points_clust.color.g = 255.0/255.0
    points_clust.color.b = 0.0/255.0

    points_clust.color.a = 1
    points_clust.lifetime = Duration()

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = Time(0)
    temppoint.point.z = 0.0

    arraypoints = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0
# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------
    while rclpy.ok():
        # -------------------------------------------------------------------------
        # Clustering frontier points
        centroids = []
        front = copy(frontiers)
        if len(front) > 1:
            ms = MeanShift(bandwidth=0.3)
            ms.fit(front)
            centroids = ms.cluster_centers_  # centroids array is the centers of each cluster

        # if there is only one frontier no need for clustering, i.e. centroids=frontiers
        if len(front) == 1:
            centroids = front
        frontiers = copy(centroids)
# -------------------------------------------------------------------------
# clearing old frontiers

        z = 0
        while z < len(centroids):
            cond = False
            temppoint.point.x = centroids[z][0]
            temppoint.point.y = centroids[z][1]

            for i in range(0, n_robots):

                transformedPoint = tf2_buffer.transform(temppoint, globalmaps[i].header.frame_id, Duration(10))
                x = array([transformedPoint.point.x, transformedPoint.point.y])
                cond = (gridValue(globalmaps[i], x) > threshold) or cond
            if (cond or (informationGain(mapData, [centroids[z][0], centroids[z][1]], info_radius*0.5)) < 0.2):
                centroids = delete(centroids, (z), axis=0)
                z = z-1
            z += 1
# -------------------------------------------------------------------------
# publishing
        arraypoints.points = []
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            arraypoints.points.append(copy(tempPoint))
        filterpub.publish(arraypoints)
        pp = []
        for q in range(0, len(frontiers)):
            p.x = frontiers[q][0]
            p.y = frontiers[q][1]
            pp.append(copy(p))
        points.points = pp
        pp = []
        for q in range(0, len(centroids)):
            p.x = centroids[q][0]
            p.y = centroids[q][1]
            pp.append(copy(p))
        points_clust.points = pp
        pub.publish(points)
        pub2.publish(points_clust)
        rate.sleep()
# -------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        node()
    except rclpy.ROSInterruptException:
        pass
