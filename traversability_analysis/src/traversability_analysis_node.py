#!/usr/bin/env python 
import time
import rospy
import math as m
import tf 
from sensor_msgs.msg import PointCloud
from data_structures.elevation_map import ElevationMap
from traversability_index.traversability_index import TraversabilityIndex
from traversability_analysis.msg import MapProp
from traversability_analysis.msg import Map
from nav_msgs.msg import OccupancyGrid
from utils_thesis.common import get_xy_limits
from utils_thesis.common import read_pcd
from utils_thesis.common import update_time_stamp
from utils_thesis.mapping import gen_emap_msg
'''
Traversability Analysis Node 
TODO 
- add function to save the traversability map to a text file  
'''
# GLOBAL CONSTANTS
DEG2RAD = m.pi/180.0
TEST1_PCD = "/home/spartan3123/catkin_ws/src/traversability_analysis/src/rosbag_generation/datasets/clear/pcd/point_cloud.pcd"

def main():
    rospy.init_node('traversability_analysis_node')
    # set parameters 
    rospy.set_param('use_sim_time', True)
    pub_emin = rospy.Publisher('e_min_viz', PointCloud)
    pub_eavg = rospy.Publisher('e_avg_viz', PointCloud)
    pub_emax = rospy.Publisher('e_max_viz', PointCloud)
    pub_tmap = rospy.Publisher('t_map', OccupancyGrid)
    pub_tmap_prop = rospy.Publisher('t_map_prop', MapProp)
    pub_emap = rospy.Publisher('emap', Map)
    # intilise elevation map and mapping constants
    [emap] = initialise(fname=TEST1_PCD)
    print "Origin Index"
    print "X: ", emap.origin_x
    print "Y: ", emap.origin_y
    print "Interpolating Elevation Map..."
    # interpolate map
    emap.interplotate(window_size=15)
    # perform tranversability analysis  
    tmap = TraversabilityIndex(emap)
    print "Evaluating Traversability Metrics on Elevation Map..."
    tmap.eval_metrics()
    print "Evaluate Traversability Index"
    tmap.eval_t_index()
    print "Generating Elevation Map Representation...."
    [e_min_viz, e_avg_cloud, e_max_cloud] = emap.get_pointcloud(meta_data=tmap.m_map)
    print "Generating Emap msg"
    emax_map_msg = gen_emap_msg(emap)
    print "Generating Traversability Map Representation "
    tmap_msg = gen_occupany_grid_msg(tmap)
    # fill in the map properties msg 
    tmap_prop_msg = gen_map_properties_msg(tmap.cell_size, tmap.emap.origin_x, tmap.emap.origin_y)
    print "Publishing msgs"
    # publish pointclouds and traversability map
    i = 0 
    while not rospy.is_shutdown(): 
        # update the time stamps (sim time could be used)
        update_time_stamp(tmap_msg)
        update_time_stamp(tmap_prop_msg)
        update_time_stamp(e_min_viz)
        update_time_stamp(e_avg_cloud)
        update_time_stamp(e_max_cloud)
        if i % 12 == 0:
            # every 60 seconds
            # anything visualised by rviz needs to be published all the time at a slow rate
            pub_emin.publish(e_min_viz)
            pub_eavg.publish(e_avg_cloud)
            pub_emax.publish(e_max_cloud)
            pub_tmap.publish(tmap_msg)
            pub_emap.publish(emax_map_msg)
        else:
            # every 5 seconds
            pub_tmap.publish(tmap_msg)
            pub_tmap_prop.publish(tmap_prop_msg)
            pub_emap.publish(emax_map_msg)   
        i = i + 1
        time.sleep(5)
        
def initialise(fname):
    # CONSTANTS 
    RESOLUTION = 0.2 #m; NOTE also set in uwb_threshold_updator_node
    # determine the x,y limits of the data set 
    [x_max, x_min, y_max, y_min] = get_xy_limits(fname)
    delta_x = x_max - x_min 
    delta_y = y_max - y_min
    # create an elevation map object
    emap = ElevationMap(delta_x=delta_x, delta_y=delta_y, cell_size=RESOLUTION, z_min=-1, z_max=5, bin_min_cap=2)
    # add points to elevation map 
    read_pcd(fname,emap)
    print "PCD Load"
    return [emap]

# MSG GENERATION FUNCTIONS 
def gen_map_properties_msg(cell_size, o_row_index, o_col_index):
    '''
        Generate a ROS map properties msg for the map object 
    '''
    map_prop = MapProp()
    map_prop.cell_size = cell_size
    map_prop.origin_row_index = o_row_index
    map_prop.origin_col_index = o_col_index
    return map_prop

def gen_occupany_grid_msg(tmap):
    '''
        Generates a ROS occupancy grid msg for a tmap object
    '''
    # constant convert degrees to radians
    DEG2RAD = m.pi/180.0
    omap = OccupancyGrid()                        
    numRows = tmap.num_rows
    numCols = tmap.num_cols
    omap.header.frame_id = "map"
    omap.info.map_load_time = rospy.get_rostime()
    omap.info.resolution = tmap.cell_size #meters
    omap.info.width = numCols  #
    omap.info.height = numRows # number of rows 

    ''' Convention for OccupancyGrid 
        Rows || to the x axis, ie the x axis indexes collumns
        Colls || to the y axis, ie the y axis indexes rows
    '''
    # determine the pose of the cell[0,0] in the map coordinate system
    [x, y] = tmap.emap.index_to_cart(0,0)
    omap.info.origin.position.x = x
    omap.info.origin.position.y = y
    omap.info.origin.position.z = -20

    # http://mathworld.wolfram.com/EulerAngles.html
    temp = tf.transformations.quaternion_from_euler(180*DEG2RAD, 0*DEG2RAD, 90*DEG2RAD) #ax, ay, az 
    # where z rotation is applied first then y 
    omap.info.origin.orientation.x = temp[0]
    omap.info.origin.orientation.y = temp[1]
    omap.info.origin.orientation.z = temp[2]
    omap.info.origin.orientation.w = temp[3]
    #load data row-major iteation of obstacle map 
    for row in tmap.t_map:
        for ti in row:
            if ti == -1:  #unknown
                omap.data.append(-1)
            else:
                omap.data.append(int(ti*100))
    ''' Figuring out coordinate convention for OccupancyGrid msg
    for i in xrange(tmap.t_map.shape[0]):
        for j in xrange(tmap.t_map.shape[1]):
            if i == 1:
                omap.data.append(100)
            else:
                if j == 1:
                    omap.data.append(0)
                else:
                    omap.data.append(-1)
    '''
    return omap

if __name__ == '__main__':
    main()


