#!/usr/bin/env python 
import rospy
import math as m
import numpy as np
import tf
import time
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from ground_map import GroundMap
from data_structures.elevation_map import ElevationMap
from uwb_fov import UWBFov
from geometry_msgs.msg import Point32
from uwb_radar_msgs.msg import UWBScan
from uwb_threshold_updater.msg import GroundSpectrum
from nav_msgs.msg import OccupancyGrid
from uwb_thresh_updater import UWBThreshUpdater
from traversability_analysis.msg import MapProp
from traversability_analysis.msg import Map
from utils_thesis.common import *
from utils_thesis.file_io import row_major_extractor
from utils_thesis.mapping import gen_emap_msg
import pyqtgraph as pg

from exit_functions import save_gamma_list

'''
UWB_Threshold_Updater_node 

TODO
- add functionality to read the traversability map from a saved text file 
'''
# GLOBAL CONSTANTS
DEG2RAD = m.pi/180.0
# Saves learned ground spectrum under the following textfile 
DATA_SET_NAME = "Clear.txt"
# location of pcd used to generate the elevation map 
TEST1_PCD = "/home/spartan3123/catkin_ws/src/traversability_analysis/src/rosbag_generation/datasets/clear/pcd/point_cloud.pcd"

class CallBackClass(object):
    def __init__(self, tf_listener, pub_uwb_viz, gmap, pub_ground_spec):
        self.tf_listener = tf_listener
        self.pub_uwb_viz = pub_uwb_viz 
        self.pub_ground_spec = pub_ground_spec
        self.gmap = gmap 
        self.uwb_fov = None
        self.ground_thresh_updater = UWBThreshUpdater(gplane_map=gmap)             
        self.tmap_loaded = False
        self.tmap_prop_loaded = False  
        self.emap_loaded = False 
        self.tmap_resolution = 0
        self.tmap = []
        self.emap = []
        self.last_update_ros_time = rospy.Time.now() 
        # intensity return visualisation performed inside the main loop
        self.intensity_avg = []
        self.intensity_max = []
        self.range_bin_indexs = []

    def ready_process_uwb_scan(self):
        '''
            Returns True if ready to process UWB Scan 
        '''
        return self.tmap_loaded == True and self.tmap_prop_loaded == True and self.emap_loaded == True

    def callback_emap(self, data):
        ''' Extracts Elevation Map in row_major order '''
        if self.emap_loaded == True:
            return 
        # load map_properties
        map_properties = {'cell_size':data.cell_size, 'origin_x_i':data.origin_row_index, 'origin_y_i':data.origin_col_index}
        error_code = data.error_code
        row_length = data.row_length
        # convert data into a 2D numpy array 
        self.emap = row_major_extractor(data.data, row_length)
        self.ground_thresh_updater.set_emap(self.emap, map_properties, error_code)
        self.emap_loaded = True

    def callback_tmap(self, data):
        ''' Extracts the traversability map loaded in row major order and
        '''
        # if tmap is already loaded do not excute callback function 
        if self.tmap_loaded == True:
            return 
        self.tmap_loaded = True
        self.tmap_resolution = data.info.resolution
        # determine the uwb fov sample points given the map resolution 
        uwb_radar = UWBFov(self.tmap_resolution)
        # generate point cloud for UWBFOV
        uwb_fov = uwb_radar.mask_indexs
        self.uwb_fov = self._gen_pointcloud_uwb_fov(uwb_fov)
        # load the tmap data np integer array -1 indicating unknown, traversabiltiy represented as a number between 0 and 1 
        self.tmap_resolution = data.info.resolution
        row_length = data.info.width
        self.tmap = row_major_extractor(data.data, row_length)
        # set tmap in the ground spectrum object 
        self.ground_thresh_updater.set_tmap(self.tmap)

    def callback_tmap_prop(self, data):
        ''' fills in the callback class map properties attributes '''
        if self.tmap_prop_loaded == False:
            self.tmap_prop_loaded = True
            self.last_update_ros_time = rospy.Time.now()
            self.tmap_resolution = data.cell_size
            self.origin_row_index = data.origin_row_index
            self.origin_col_index = data.origin_col_index
            map_properties = {'cell_size':data.cell_size, 'origin_x_i':data.origin_row_index, 'origin_y_i':data.origin_col_index} 
            self.ground_thresh_updater.set_map_properties(map_properties) 
        else:
            return

    def callback_uwb(self, data):
        ''' Processes the UWB Radar Return '''
        # print data.header.seq
        # detection of negative time step
        dt = (rospy.Time.now() - self.last_update_ros_time)
        self.last_update_ros_time = rospy.Time.now()
        if dt.to_sec() < 0:
            print "Negative timestep Clearing tf buffer" 
            self.tf_listener.clear()
            time.sleep(10.0) # fill buffer up
            print "Cleared Buffer"
            return
        if self.ready_process_uwb_scan() == True:
            # generate a pointcloud to represent the UWB FOV mask 
            uwb_mask_pcd = self.uwb_fov
            # update timestamp of UWB_Radar msg to get most recent transform (timestamp is set to 0.0)
            update_time_stamp_zero(uwb_mask_pcd)
            try: # and transform uwb_mask into the global frame 
                # wait for transform  
                trans_flag = False
                timeout_counter = 0 
                trans_flag = self.tf_listener.canTransform("/map", "/body", uwb_mask_pcd.header.stamp)
                while (trans_flag == False and timeout_counter < 10 ): 
                    trans_flag = self.tf_listener.canTransform("/map", "/body", uwb_mask_pcd.header.stamp)
                    timeout_counter = timeout_counter + 1 
                    print "waiting...."
                    time.sleep(0.01)
                if trans_flag == False:
                    print "cannot transform uwb radar scan"
                    return  # discard uwb_scan
                assert(self.uwb_fov.channels[0].values[198] < 6.9) # checking range of mask point 
                # transfrom pcd into the global frame
                uwb_mask_pcd = self.tf_listener.transformPointCloud("/map", uwb_mask_pcd)
            except tf.LookupException as e:
                print str(e)
                return
            except tf.ConnectivityException as e:
                print str(e)
                return
            except tf.ExtrapolationException as e:
                print str(e)
                return
            # use UWB Radar scan to update the ground intensity spectrum profile 
            self.ground_thresh_updater.update_ground_thresh(data, uwb_mask_pcd)
            # publish uwb_fov_mask 
            self.pub_uwb_viz.publish(uwb_mask_pcd)
            # visualise learned ground intensity spectrum
            if data.header.seq % 8 == 0:
                # get the simulated intensity sectrum data
                # plotting of the intensity spectrum is done in the main loop every 5 seconds
                [inten_avg, inten_max, bin_indexs] = self.ground_thresh_updater.visualise_ground_spectrum(uwb_mask_pcd)
                self.intensity_avg = inten_avg
                self.intensity_max = inten_max
                self.range_bin_indexs = np.array(bin_indexs)*0.3
                # generate ground spectrum msg from the learned ground spectrum histogram  
                grnd_spec_msg = self._gen_ground_spec_msg(self.ground_thresh_updater.get_ground_histogram())
                # publish the current ground spectrum 
                self.pub_ground_spec.publish(grnd_spec_msg)

            # learn the ground returns from the uwb scan

    @staticmethod
    def _add_to_channels(range2p, az, elv, ch_r, ch_a, ch_e, ch_msk):
        '''
            Add data into the channels for the UWB_FOV pointcloud  
        '''
        ch_r.values.append(range2p)
        ch_a.values.append(az)
        ch_e.values.append(elv)
        ch_msk.values.append(0) 

    @staticmethod
    def _gen_point(x, y, pointcloud):
        '''
            Generate point and adds it to the UWB_FOV pointcloud 
        '''
        point_msg = Point32()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = 0
        pointcloud.points.append(point_msg)

    @staticmethod
    def _gen_pointcloud_uwb_fov(points):
            uwb_fov_pc = PointCloud()
            # meta data channels 
            ch_range = ChannelFloat32()
            ch_range.name = "range"
            ch_azi = ChannelFloat32()
            ch_azi.name = "azimuth"
            ch_elv = ChannelFloat32()
            ch_elv.name = "elevation"
            ch_msk = ChannelFloat32()
            ch_msk.name = "mask_state"
            # extract x,y and meta_data from the sensor_FOV mask
            # each point is of the form [x,y, range, azimuth, elevation]
            xpts = [point[0] for point in points]
            ypts = [point[1] for point in points]
            ranges = [point[2] for point in points]
            azis = [point[3] for point in points]
            elvs = [point[4] for point in points]
            # add points to pointcloud
            for i in xrange(len(xpts)):
                CallBackClass._gen_point(xpts[i],ypts[i], uwb_fov_pc)
                CallBackClass._add_to_channels(ranges[i], azis[i], elvs[i], ch_range, ch_azi, ch_elv, ch_msk)
            # add headers to pointcloud msg
            uwb_fov_pc.header.frame_id = "/body"
            # append channels to pointcloud msg
            uwb_fov_pc.channels.append(ch_range)
            uwb_fov_pc.channels.append(ch_azi)
            uwb_fov_pc.channels.append(ch_elv)
            uwb_fov_pc.channels.append(ch_msk)
            return uwb_fov_pc

    @staticmethod
    def _gen_ground_spec_msg(ground_spectrum):
        '''
            Given a ground spectrum histogram, format: dictionary 
            indexed by the range bin index, the value being a list containing gamma statistics
            [count, mean_gamma, mean_gamma_sqrd, std_gamma] 

            a gamma value will only be present if there is atleast one gamma update for the range bin

            return a GroundSpectrum Msg 
        '''
        # iterate over the ground sepectrum histogram and create the msg 
        range_bin_indexs = []
        gamma_avgs = []
        gamma_stds = []
        count = []
        grnd_spec_msg = GroundSpectrum()
        for range_bin_index, gamma_vector in ground_spectrum.iteritems():
            range_bin_indexs.append(range_bin_index)
            gamma_avgs.append(gamma_vector[1])
            gamma_stds.append(gamma_vector[3])
            count.append(gamma_vector[0])

        grnd_spec_msg.range_bin_index = range_bin_indexs
        grnd_spec_msg.gamma_avg = gamma_avgs
        grnd_spec_msg.gamma_std = gamma_stds
        grnd_spec_msg.count = count

        return grnd_spec_msg

## end callback class 

def main():
    rospy.init_node('uwb_threshold_updater_node')
    # set any required ROS parameters 
    rospy.set_param('use_sim_time', True)
    #----init tf listener and all Publishers-------------------------------------------
    # tf transform listener 
    listener = tf.TransformListener(True, rospy.Duration(10.0)) 
    # publisher for ground plane visualisation
    pub_gplane_viz = rospy.Publisher('e_gp_viz', PointCloud)
    # publisher for the ground plane elevation map 
    pub_gp_emap = rospy.Publisher('gp_emap', Map)
    # publisher to visualise UWB FOV
    pub_uwb_fov = rospy.Publisher('uwb_fov', PointCloud)
    # publisher for ground_spectrum msg for the data fusion node 
    pub_ground_spec = rospy.Publisher('ground_spec', GroundSpectrum)
    #------------------------------------------------------------------------------------
    # intilise low resolution elevation map for ground plane fitting
    [gp_emap] = init_emap(fname=TEST1_PCD) # should set through comandline argument
    # interploate the elvation map 
    gp_emap.interplotate(5)
    # generate gp_emap msg for data fusion node 
    emap_msg = gen_emap_msg(gp_emap)
    # fit ground plane
    gmap = GroundMap(gp_emap)
    gmap.ground_fit()
    # instiate callback class
    callback_manager = CallBackClass(listener, pub_uwb_fov, gmap, pub_ground_spec)
    #----init subscribers --------------------------------------------------------------------
    # subscriber to UWB radar scans
    rospy.Subscriber("radar_scan", UWBScan, callback_manager.callback_uwb)
    # subscriber to Traversability Map 
    rospy.Subscriber("t_map", OccupancyGrid, callback_manager.callback_tmap)
    rospy.Subscriber("t_map_prop", MapProp, callback_manager.callback_tmap_prop)
    # subscriber to high resolution elevation[cm] map from traversability analysis node 
    rospy.Subscriber("emap", Map, callback_manager.callback_emap)
    #-----------------------------------------------------------------------------------------
     # ground plane visualisation 
    [gp_emin_cloud, gp_eavg_cloud, gp_emax_cloud] = gp_emap.get_pointcloud(meta_data=gmap.m_map)
    print "Publishing msgs"
    # publish pointclouds and traversability map 
    PW = pg.plot()
    while not rospy.is_shutdown(): 
        update_time_stamp(gp_emax_cloud)
        pub_gplane_viz.publish(gp_emax_cloud)
        pub_gp_emap.publish(emap_msg)
        # plot the learned ground spectrum assuiming completly flat terrain
        if len(callback_manager.intensity_avg) > 0:
            PW.plot(callback_manager.intensity_avg, x=callback_manager.range_bin_indexs, clear=True)
            PW.plot(callback_manager.intensity_max, x=callback_manager.range_bin_indexs, clear=False)
            pg.QtGui.QApplication.processEvents()
        time.sleep(5)
    # node shutdown procedure 
    print "Saving learned statistics"
    # saves the gamma list to the current dictory
    # save_gamma_list(callback_manager.ground_thresh_updater.ground_spectrum.gamma_list)
    print "Saved Files"

def init_emap(fname):
    '''
        Intilises the elevation map, which will be used for ground clutter rejection  
    '''
    # CONSTANTS 
    CELL_SIZE = 1 # m
    # determine the x,y limits of the data set 
    [x_max, x_min, y_max, y_min] = get_xy_limits(fname)
    delta_x = x_max - x_min 
    delta_y = y_max - y_min
    # create an elevation map object - elevation map with large cell size will be apart of the outlier rejection
    # to help decide which elevation values belong to the ground plane and which elevation values are apart of obstacles
    # on the ground 
    emap = ElevationMap(delta_x=delta_x, delta_y=delta_y, cell_size=CELL_SIZE, z_min=-1, z_max=5, bin_min_cap=2)
    # add points to elevation map 
    read_pcd(fname,emap)
    print "Finished PCD load"
    return [emap]

if __name__ == '__main__':
    main()


