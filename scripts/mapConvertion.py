#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from octomap_msgs.msg import Octomap
import octomap

def octomap_callback(msg):
    rospy.loginfo("Received octomap message, processing...")

    octree = octomap.OcTree(msg.data)
    
    # Define OccupancyGrid
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = Header(stamp=rospy.Time.now(), frame_id='map')
    occupancy_grid.info.resolution = msg.resolution

    # Convert Octree to OccupancyGrid
    for leaf in octree.iter_leafs():
        x, y, z = leaf.get_coordinate()
        if z < z_threshold: # Consider the floor and objects
            i = int((x - min_x) / msg.resolution)
            j = int((y - min_y) / msg.resolution)
            occupancy_grid.data[j * width + i] = 100 if leaf.get_value() > occupancy_threshold else 0

    # Publish the occupancy grid
    occupancy_grid_pub.publish(occupancy_grid)

if __name__ == "__main__":
    rospy.init_node('octomap_to_gridmap')

    # Define parameters
    min_x = rospy.get_param("~min_x", -10)
    min_y = rospy.get_param("~min_y", -10)
    width = rospy.get_param("~width", 20)
    height = rospy.get_param("~height", 20)
    z_threshold = rospy.get_param("~z_threshold", 0.5)  # Ignore everything below this threshold
    occupancy_threshold = rospy.get_param("~occupancy_threshold", 0.5)

    # Initialize the grid map
    occupancy_grid = OccupancyGrid()
    occupancy_grid.info.width = width
    occupancy_grid.info.height = height
    occupancy_grid.info.resolution = 0.05  # Default value, will be overwritten
    occupancy_grid.data = [0 for _ in range(width * height)]  # Initialize with free space

    # Subscribe and publish
    rospy.Subscriber('/octomap_full', Octomap, octomap_callback)
    occupancy_grid_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)

    rospy.spin()