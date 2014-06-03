#!/usr/bin/env python 
import math as m
import numpy as np
from nav_msgs.msg import OccupancyGrid
from traversability_analysis.msg import MapProp
from traversability_analysis.msg import Map
from utils_thesis.common import *
from utils_thesis.mapping import cartesian_2_index
from utils_thesis.lin_alg import angle_between_vectors
from utils_thesis.scaling_functions import inverse_parabola
from sensor_model import SensorModel


'''
    uwb_data fusion class 
    measures the probability that the ground is occupied by an untraversable object 
'''
# GLOBAL CONSTANTS
DEG2RAD = m.pi/180.0

class UWBDataFusion(object):
    def __init__(self, init_p_occ=0.8, min_count=5):

        self.tmap_bin = None
        self.tmap = None 
        self.tmap_aug = None 
        self.pmap = None
        self.map_properties = {}
        self.tmap_loaded = False 

        self.gplane_loaded = False
        self.gp_plane_map = None
        self.gp_emap = None
        self.gp_emap_prop = {}
        self.gp_ec = 0
        # gamma model 
        self.gamma_model_loaded = False 
        self.gamma_rbin_ids = []
        self.gamma_avgs = []
        self.gamma_stds = []
        self.count = []
        # intensity spectrum of ground
        self.ground_spectrum = {}
        self.min_count = min_count

        # probability model variables
        self.init_p_occ = init_p_occ
        # saftey factor - the largest this should be is 1 - init_p_occ
        self.saftey_value = 1 - self.init_p_occ

        # sensor model 
        self.sensor_model = SensorModel()

    def set_tmap(self, tmap, map_properties):
    	'''
    		Set the traversability map
    		Input: Tmap, npfloat array ti is [0-1] negative value indicates unknown ti 
    	'''
    	self.map_properties = map_properties
        # initial traversability map 
    	self.tmap = tmap 
    	# determine the occupancy map - binary mask where ti value is 1  
    	self.tmap_bin = (tmap == 1)*1
        # init probability of occupancy map 
        self.pmap = self.tmap_bin*self.init_p_occ
        # init tmap_aug
        self.agument_tmap()
        # set the initilised flag 
        self.tmap_loaded = True

    def set_gplane(self, gplane, gplane_emap, gp_emap_prop, gp_ec):
        '''
            Input:
                -gplane GroundPlane object
                -gplane_emap numpy elevation map, in cm 
                -gp_emap_prop dictionary containing the map map_properties
                -gp_ec the error code for the elevation map in m; max_z value + 1 
        '''
        self.gp_plane_map = gplane
        self.gp_emap = gplane_emap
        self.gp_emap_prop = gp_emap_prop
        self.gp_ec = gp_ec
        self.gplane_loaded = True
        assert('cell_size' in gp_emap_prop)


    def update_pocc(self, uwb_mask_pcd, uwb_scan ):
        
        if self.init() == False:
            return 
        # update ground spectrum
        self.determine_ground_spec(uwb_mask_pcd)
        # evaluate measurement likelyhood 
        self.sensor_model.prob_sen(uwb_mask_pcd, self.ground_spectrum, uwb_scan)
        # update pocc
        self.bayesian_update(uwb_mask_pcd)

    def determine_ground_spec(self, uwb_mask_pcd, radar_range_res=0.3):
        '''
            Determine the ground spectrum vector for the current ground 
            Input
                -uwb_pcd, pointcloud representation of where the uwb_beamwidth is, relative to the map frame 

            Returns: Dictionary, contianing the ground intensity values indexed by the range bin 
                     Note not all range bins will have valid intensity values, if unknown -1 -1 will
                     be present 
 
        '''
        if self.init() == False:
            return 0
        ranges = uwb_mask_pcd.channels[0].values
        # angle scalar for the pdd
        angle_scalars = self._evaluate_angle_scalar(uwb_mask_pcd)
        # determine the sensor vector 
        s_vec = self._evalaute_sensor_vector(uwb_mask_pcd)
        # get the incident angle mask 
        angle_mask = self._evaluate_incident_angle(uwb_mask_pcd, s_vec)
        # determine the incident angles and weights for each range gate 
        angle_weights_gated = {}
        # range gate the angles and determine the expected ground return for each range bin 
        for i, angle in enumerate(angle_mask):
            if i > 0:
                bin_index =  int(m.floor(ranges[i]/radar_range_res))
                if bin_index in angle_weights_gated:
                    angle_weights_gated[bin_index].append((angle, angle_scalars[i]))
                else:
                    angle_weights_gated[bin_index] = [(angle, angle_scalars[i])]

        # for each range bin evaluate the expected ground return  [iavg, imax, imin]
        # if unknown -1
        intensity_spectrum = {}
        for bin_index, angles_weights in angle_weights_gated.iteritems():
            # only request intensity if there are valid incident angles
            intensity_spectrum[bin_index] = self.expected_ground_return(angles_weights, bin_index)
        self.ground_spectrum = intensity_spectrum
        return 1

    def bayesian_update(self, uwb_pcd):
        
        points = uwb_pcd.points
        ps_occ = uwb_pcd.channels[9].values
        ps_empty = uwb_pcd.channels[10].values
        region = uwb_pcd.channels[5].values

        for i, point in enumerate(points):
            if i > 0:
                if region[i] != -1 and ps_occ[i] != -1 and ps_empty[i] != -1:
                    [row_index, col_index] = cartesian_2_index(point.x, point.y, self.map_properties)
                    pri = self.pmap[row_index, col_index]
                    # bayesian update
                    pocc = (ps_occ[i]*pri)/(ps_occ[i]*pri + ps_empty[i]*(1-pri))
                    assert(pocc <= 1)
                    self.pmap[row_index, col_index] = pocc

        # update tmap
        self.agument_tmap()

    def agument_tmap(self):
        # apply a saftey factor to evaluate the safe probability of occupancy map
        pocc_sf = self.pmap + self.saftey_value
        # apply piece wise function to saturate the probabilities 
        procc_sf = np.piecewise(pocc_sf, [pocc_sf <= 1, pocc_sf > 1], [lambda pocc_sf:pocc_sf , 1])
        # calculate the augmented traversability map 
        self.tmap_aug = self.tmap*procc_sf

    def init(self):
        return self.tmap_loaded == True and self.gplane_loaded == True and self.gamma_model_loaded == True


    def update_ground_model(self, rbin_ids, gamma_avgs, gamma_stds, counts):
        '''
            Update the ground gamma model. A gamma value will only be present if there has been at least one update to 
            a range bin 
        '''
        self.gamma_model_loaded = True
        self.gamma_rbin_ids = rbin_ids 
        self.gamma_avgs = gamma_avgs
        self.gamma_stds = gamma_stds
        self.count = counts

    def _evalaute_sensor_vector(self, uwb_fov_mask):
        '''
            Evaluates the senor vector, which is a vector from the sensor orging to a point on the ground
            which is projected from the x,y positions of the UWB_FOV_Mask to the ground plane evaluate using 1x1m grids.

            Given UWB_MASK format sensor_msgs.Pointcloud, encoded with meta data such as state.
            evaluate the sensing vector and update the range channel of the UWB_MASK to the true_range 
        '''
        points = uwb_fov_mask.points
        sensor_pos = points[0]
        S = np.array([sensor_pos.x, sensor_pos.y, sensor_pos.z])
        zero_vector = np.array([0.0,0.0,0.0])
        s_list = []
        s_list.append(zero_vector)
        for i, point in enumerate(points):
            if i > 0:
                # project the mask point into the ground using the higher resolution elevation map
                x_loc = point.x
                y_loc = point.y
                [row_index, col_index] = cartesian_2_index(x_loc, y_loc, self.gp_emap_prop)
                # determine the z projection of the mask point in meters 
                z_proj = self.gp_emap[row_index, col_index]/100.0 
                # check z value of cell is known
                if z_proj == self.gp_ec:
                    s_list.append(zero_vector)
                else:
                    assert(z_proj != self.gp_ec)
                    # therefore the point on the ground is 
                    G = np.array([x_loc, y_loc, z_proj])
                    # determine the sensing vector - originates from ground
                    # vector from ground point to sensor origin
                    v_gs = S - G
                    s_list.append(v_gs)

        return s_list

    def _evaluate_incident_angle(self, uwb_fov_mask, s_list):
        '''
            Evaluates the incident angle between the sensing vector and the ground 

            Input: sensing vector, list of numpy vectors 
            the first sensing vector is a zero vector, othe rest of the data vectors
            that orginate from the ground and travel to the radar position , unless the elevation value 
            is unknown 
        '''
        zero_vector = np.array([0.0,0.0,0.0])
        points = uwb_fov_mask.points
        i_angles = []
        # to align the angles vector with all the points
        i_angles.append(-1)
        for i, point in enumerate(points):
            if i > 0:
                sensor_vector = s_list[i]
                if (sensor_vector != zero_vector).all():
                    x_loc = point.x
                    y_loc = point.y
                    normal_vector =  self.gp_plane_map.get_normal(x_loc, y_loc)
                    # check normal vector is known 
                    if (normal_vector == zero_vector).all():
                         i_angles.append(-1)
                    else:
                        aspect_angle = angle_between_vectors(normal_vector, sensor_vector)
                        incident_angle = m.fabs(90 - aspect_angle)
                        i_angles.append(incident_angle)
                else:
                    i_angles.append(-1)
        return i_angles

    def _evaluate_angle_scalar(self, uwb_fov_mask, uwb_beamwidth=40):
        '''
            Evaluates a scalar that compensates for the intensity variation in the UWB radar beam pattern  
        '''
        RAD2DEG = 180.0/m.pi
        points = uwb_fov_mask.points
        azi_channel = uwb_fov_mask.channels[1].values
        angle_scalars = []
        # place holder scalar for the sensor origin point
        angle_scalars.append(0)
        for i, point in enumerate(points):
            if i > 0:
                # get azimuth position in degrees  
                azimuth = azi_channel[i]*RAD2DEG
                a_scalar = inverse_parabola(azimuth, uwb_beamwidth)
                angle_scalars.append(a_scalar)

        return angle_scalars

    @staticmethod
    def eval_incident_angle_sum(angle_weights):
        '''
            input: angle_weights, a list of tupples containing an incident angle and an assoicated weight
                   angle, in degrees -1 if the incident angles is unknown

            returns: sum of the incident angle multiplied by its weight or -1 if there are invalid anlges in 
                     the list of angle_weights
        '''
        incident_angle_sum = 0
        DEG2RAD = m.pi/180.0
        for i, angle_weight in enumerate(angle_weights):
            angle, weight = angle_weight[0]*DEG2RAD, angle_weight[1]
            if angle_weight[0] == -1:
                return -1
            assert(weight > 0)
            incident_angle_sum = incident_angle_sum + m.sin(angle)*weight
            assert(m.sin(angle)*weight > 0)
        return incident_angle_sum

    def expected_ground_return(self, angle_weights, range_bin_index):
        '''
            input: angle_weights, a list of tupples containing an incident angle and an assoicated weight
            angle, in degrees -1 if the incident angles is unknown
        '''
        incident_angle_sum = self.eval_incident_angle_sum(angle_weights)
        # check the current mask does not lie in an unknown region 
        if incident_angle_sum == -1:
            return [-1, -1]
        max_index = len(self.count) - 1 
        index_offset = self.gamma_rbin_ids[0]
        # index used to acess the gamma vectors which are aligned to particular range bins 
        index = range_bin_index - index_offset
        # check there are measurements are in a valid range bin
        if index > max_index or index < 0:
            return [-1, -1]
        else:
            if self.count[index] >= self.min_count:
                mean_gamma = self.gamma_avgs[index]
                std_gamma = self.gamma_stds[index]
                ground_return = mean_gamma*incident_angle_sum
                ground_return_max = (mean_gamma + std_gamma)*incident_angle_sum
                return [ground_return, ground_return_max]
            else:
                return [-1, -1]
        
