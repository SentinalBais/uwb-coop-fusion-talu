#!/usr/bin/env python 
import rospy
import math as m
import tf
import time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
from traversability_analysis.msg import MapProp
from traversability_analysis.msg import Map
from uwb_radar_msgs.msg import UWBScan
from utils_thesis.mapping import gen_occupany_grid_msg_np
from utils_thesis.common import *
from utils_thesis.file_io import row_major_extractor
from utils_thesis.file_io import row_major_extractor_float
from data_structures.elevation_map import ElevationMap
from uwb_threshold_updater.uwb_fov import UWBFov
from uwb_threshold_updater.ground_map import GroundMap
from uwb_threshold_updater.msg import GroundSpectrum
from uwb_data_fusion import UWBDataFusion
import numpy as np
import pyqtgraph as pg

'''
uwb_data fusion node 

'''
# GLOBAL CONSTANTS
DEG2RAD = m.pi/180.0

class CallBackClass(object):
    def __init__(self, tf_listener, pub_tmap_aug, pub_tmap_viz, pub_fov ):

        # tf listener 
        self.tf_listener = tf_listener
        # Publishers 
        self.pub_tmap_aug = pub_tmap_aug
        self.pub_tmap_viz = pub_tmap_viz
        self.pub_fov = pub_fov
        # uwb FOV mask 
        self.uwb_fov = None
        # uwb data fusion object
        self.ground_spec_loaded = False
        self.data_fuser = UWBDataFusion()
        # traversability map member variables 
        self.tmap_loaded = False
        self.tmap_prop_loaded = False
        self.tmap_map_properties = {}
        self.tmap = []
        # detection of negative time step 
        self.last_update_ros_time = rospy.Time.now() 
        # alternative visualisation of augmented traversability map 
        self.emax_pcd_loaded = False
        self.emax_pcd = None 
        # flag for ground plane map object being loaded
        self.gpmap_loaded = False
        #
        # data fusion object 
        # holds the agumented traversabilty map
        # and functions to updated the agumented traversability map  


    def uwb_callback(self, data):
        
        # detection of negative time step
        dt = (rospy.Time.now() - self.last_update_ros_time)
        self.last_update_ros_time = rospy.Time.now()
        if dt.to_sec() < 0:
            print "Negative timestep Clearing tf buffer" 
            self.tf_listener.clear()
            time.sleep(10.0)
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
            self.data_fuser.update_pocc(uwb_mask_pcd, data)
            self.pub_fov.publish(uwb_mask_pcd)

    def ground_spectrum_callback(self, data):
        '''
            Updates the ground spectrum model 
        '''
        # update the ground spectrum model 
        self.data_fuser.update_ground_model(data.range_bin_index, data.gamma_avg, data.gamma_std, data.count)
        self.ground_spec_loaded = True

    def tmap_callback(self, data):
        '''
            Extracts the traversability map loaded in row major order. Callback should only execute once
        '''
        # if tmap is already loaded do not excute callback function 
        if self.tmap_loaded == True:
            return 
        self.tmap_resolution = data.info.resolution
        # determine the uwb fov sample points given the map resolution 
        uwb_radar = UWBFov(self.tmap_resolution)
        # generate point cloud for UWBFOV
        uwb_fov = uwb_radar.mask_indexs
        # generate channels for which will encode the sensor model scalars and the probabilities 
        self.uwb_fov = self._gen_pointcloud_uwb_fov(uwb_fov)
        # load the tmap data 
        row_length = data.info.width
        tmap = row_major_extractor_float(data.data, row_length)
        # tmap a float nparray scaled between 0 and 1, negative value if value is unknown 
        self.tmap = tmap/100.0
        self.tmap_loaded = True
        # load the tmap and the tmap properties into the data fusion object 
        if self.tmap_prop_loaded == True and self.tmap_loaded == True:
            self.data_fuser.set_tmap(self.tmap, self.tmap_map_properties)

    def tmap_prop_callback(self, data):
        ''' 
            Extracts traversability map, map properties. Callback should only execute once 
        '''
        if self.tmap_prop_loaded == True:
            return
        self.last_update_ros_time = rospy.Time.now()
        self.tmap_map_properties = {'cell_size':data.cell_size, 'origin_x_i':data.origin_row_index, 'origin_y_i':data.origin_col_index} 
        self.tmap_prop_loaded = True
        # load the tmap and the tmap properties into the data fusion object 
        if self.tmap_prop_loaded == True and self.tmap_loaded == True:
            self.data_fuser.set_tmap(self.tmap, self.tmap_map_properties)


    def gpmap_callback(self, data):
        ''' 
            Ground plane elevation map is required for the estimation of the elevation scalar. The pcd mask points are 
            projected down on to the elevations specified by this map. This map is used to estimate incident angles to the ground
            in order to estimate the ground return 

            The callback function is only executed once 
        '''
        if self.gpmap_loaded == True:
            return 
        # load map_properties
        map_properties = {'cell_size':data.cell_size, 'origin_x_i':data.origin_row_index, 'origin_y_i':data.origin_col_index}
        error_code = data.error_code
        row_length = data.row_length
        # convert data into a 2D numpy array 
        gp_emap = row_major_extractor(data.data, row_length)
        gp_emap_prop = map_properties
        gp_emap_error_code = error_code
        self.gpmap_loaded = True
        # perform ground plane fitting and instantiate a ground_map object
        # instantiate and elevation map object using numpy elevation map defined in cm and error code defined in m (whole number)
        emap = ElevationMap(emap=gp_emap, cell_size=data.cell_size, o_row_i=data.origin_row_index, o_col_i=data.origin_col_index, ec=data.error_code)
        self.gp_plane_map = GroundMap(emap)
        self.gp_plane_map.ground_fit()
        print "loaded ground plane map"
        # setting the ground plane map and the ground plane elevation map in the data fuser object 
        self.data_fuser.set_gplane(self.gp_plane_map, gp_emap, gp_emap_prop, gp_emap_error_code)

   
    def ready_process_uwb_scan(self):
        return self.tmap_loaded == True and self.tmap_prop_loaded == True and self.gpmap_loaded == True and self.ground_spec_loaded == True

    def ground_plane_pcd_callback(self, data):
        '''
            Loads the elevation map pointcloud representation (high resolution elevation map required)
        '''
        if self.emax_pcd_loaded == True:
            return 
        self.emax_pcd_loaded = True 
        self.emax_pcd = data


    def generate_tmap_viz_pcd(self):
        '''
            Encodes the augmented traversability map values into a pointcloud 
            returns a pointcloud msg publishing occurs in the main loop 
        '''
        # todo
        pass

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
            # scalars for sensor model 
            ch_a = ChannelFloat32()
            ch_a.name = "angle_scalar"
            ch_b = ChannelFloat32()
            ch_b.name = "region"
            ch_c = ChannelFloat32()
            ch_c.name = "power_scalar"
            ch_d = ChannelFloat32()
            ch_d.name = "power_disp"
            ch_e = ChannelFloat32()
            ch_e.name = "distance_scalar"
            ch_f = ChannelFloat32()
            ch_f.name = "pm_given_occ"
            ch_g = ChannelFloat32()
            ch_g.name = "pm_given_empty"
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
                ch_a.values.append(-1)
                ch_b.values.append(-1)
                ch_c.values.append(-1)
                ch_d.values.append(-1)
                ch_e.values.append(-1)
                ch_f.values.append(-1)
                ch_g.values.append(-1)
            # add headers to pointcloud msg
            uwb_fov_pc.header.frame_id = "/body"
            # append channels to pointcloud msg
            uwb_fov_pc.channels.append(ch_range) # 0
            uwb_fov_pc.channels.append(ch_azi)   # 1
            uwb_fov_pc.channels.append(ch_elv)   # 2
            uwb_fov_pc.channels.append(ch_msk)   # 3
            uwb_fov_pc.channels.append(ch_a)     # 4 angle 
            uwb_fov_pc.channels.append(ch_b)     # 5 region
            uwb_fov_pc.channels.append(ch_c)     # 6 power 
            uwb_fov_pc.channels.append(ch_d)     # 7 power disp
            uwb_fov_pc.channels.append(ch_e)     # 8 distance 
            uwb_fov_pc.channels.append(ch_f)     # 9 pm|occ
            uwb_fov_pc.channels.append(ch_g)     # 10 pm|empty
            return uwb_fov_pc

## end callback class 

def main():
    rospy.init_node('uwb_data_fusion_node')
    # set any required ROS parameters 
    rospy.set_param('use_sim_time', True)
    # -------- init tf listener and all Publishers------------------------------------------------------
    listener = tf.TransformListener(True, rospy.Duration(10.0)) 
    pub_tmap_aug = rospy.Publisher('tmap_aug', OccupancyGrid) 
    pub_tmap_aug_viz = rospy.Publisher('tmap_aug_viz', PointCloud)
    pub_uwb_fov = rospy.Publisher('uwb_fov2', PointCloud)
    # --------------------------------------------------------------------------------------------------
    # instantiate callback class 
    callback_manager = CallBackClass(tf_listener=listener, pub_tmap_aug=pub_tmap_aug, pub_tmap_viz=pub_tmap_aug_viz, pub_fov=pub_uwb_fov)
    #----------init subscribers -------------------------------------------------------------------------
    rospy.Subscriber("radar_scan", UWBScan, callback_manager.uwb_callback)
    # subscriber to Traversability Map 
    rospy.Subscriber("t_map", OccupancyGrid, callback_manager.tmap_callback)
    rospy.Subscriber("t_map_prop", MapProp, callback_manager.tmap_prop_callback)
    # subscriber to low resolution elevation[cm] map from uwb_threshold_analyser node 
    rospy.Subscriber("gp_emap", Map, callback_manager.gpmap_callback)
    # subscriber to the ground spectrum
    rospy.Subscriber("ground_spec", GroundSpectrum, callback_manager.ground_spectrum_callback)
    # ----------------------------------------------------------------------------------------------------
    PW = pg.plot()
    while not rospy.is_shutdown(): 
        # publish the agumented traversability map both in occupancy grid form and pointcloud form
        if callback_manager.ready_process_uwb_scan() == True:
            # publish the augmented tmap 
            tmap_aug = gen_occupany_grid_msg_np(callback_manager.data_fuser.tmap_aug, callback_manager.data_fuser.map_properties)
            pub_tmap_aug.publish(tmap_aug)
            ground_spec = callback_manager.data_fuser.ground_spectrum
            if len(ground_spec.values()) > 0:
                grn_spec = ground_spec.values()
                mean_values = [value[0] for value in grn_spec]
                x = np.array(ground_spec.keys(),dtype=np.float32)*0.3
                PW.plot(mean_values, x=x, clear=True)
                pg.QtGui.QApplication.processEvents()
        # sleep 
        time.sleep(1)
    # node shutdown procedure 

if __name__ == '__main__':
    main()


