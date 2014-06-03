
from utils_thesis.mapping import cartesian_2_index
from utils_thesis.lin_alg import angle_between_vectors
from utils_thesis.scaling_functions import inverse_parabola
from sensor_msgs.msg import ChannelFloat32
from ground_spectrum import GroundSpectrum
import numpy as np
import math as m

'''
    Issues, when sensor 
'''

#CONSTANTS
MASKABLE = 100

class UWBThreshUpdater(object):
    def __init__(self, gplane_map=None, uwb_beam_width=40, uwb_range_res=0.3):
        '''
            Arguments 
            - Tmap is a 2D numpy array representing traversability 
              (not optional)
        '''

        ''' (elevation map coordinate frame)
        (x axis)
        ^
        |
        |
        *_________> (y axis)
        '''

        # uwb_update properties - should set these properties in a launch file or parameter server 
        self.offset = 1.5 #m
        self.initlized = False 
        self.uwb_beam_width = uwb_beam_width 
        self.uwb_range_res = uwb_range_res
        self.ground_spectrum = GroundSpectrum()
        # initilise placeholder arguments 
        self.tmap = None
        self.map_properties = None
        # ground plane 
        self.gplane_map = gplane_map
        # higher resolution elevation map 
        self.emap = None
        self.emap_properties = None
        self.emap_error_code = None


    def set_emap(self, emap, map_properties, error_code):
        '''
            Given elevation map in centimeters, the elevation map should contain the lowest
            elevations for each cell 
        '''
        self.emap = emap
        self.emap_properties = map_properties
        self.emap_error_code = error_code
        self.initlized = self._intilized_status()

    def set_tmap(self, tmap):
        self.tmap = tmap
        self.initlized = self._intilized_status()

    def set_map_properties(self, map_properties):
        self.map_properties = map_properties
        self.cell_size = map_properties['cell_size']
        self.origin_x_i = map_properties['origin_x_i']
        self.origin_y_i = map_properties['origin_y_i']
        self.initlized = self._intilized_status()

    def set_gplane_map(self, gplane_map):
        self.gplane_map = gplane_map
        self.initlized = self._intilized_status()

    def get_ground_histogram(self):
        return self.ground_spectrum.ground_histogram

    @staticmethod
    def _missing_args(tmap, map_properties, gplane_map, emap):
        '''
            Returns true if the current object cant be intilized properly 
        '''
        return tmap == None or map_properties == None or gplane_map == None or emap == None

    def _intilized_status(self):
        '''
            Returns True if the current object is initilised and updates the current object status
        '''
        self.initlized = not UWBThreshUpdater._missing_args(self.tmap, self.map_properties, self.gplane_map, self.emap)
        return self.initlized

    def update_ground_thresh(self, uwb_scan, uwb_fov_mask):
        '''
            given UWB scan and a mask of points representing the UWBFov in the global refference frame
            update the ground plane thresholds. The uwb_fov_mask also contains the position of the UWB
            radar. format: sensor_msgs.PointCloud  

            input: uwb_fov_maks, datatype sensor_msgs pointcloud 
            encoded with x,y,0 range, azimuth elevation position of the uwb_mask
        '''
        if self.initlized == False:
            return
        # traverse the UWB FOV and work out the largest range in which there are no obstacles 
        max_range = self._furthest_bin(uwb_fov_mask)
        # determine the evaluate the sensor vector
        s_list = self._evalaute_sensor_vector(uwb_fov_mask)
        # update the mask state channel, which indicates which points in the UWB FOV mask are being 
        # used to update the ground spectrum  
        self._update_uwb_fov_msk_state(uwb_fov_mask, max_range)
        # determine incident angles (sensor vector and true range is evaluated here)
        # this will effect the range gatting of mask points
        inc_angles = self._evaluate_incident_angle(uwb_fov_mask, s_list)
        # get the angle scalars for the masks
        angle_scalar = self._evaluate_angle_scalar(uwb_fov_mask)
        # get the elevation scalars for the mask 
        # TODO 
        points = uwb_fov_mask.points
        # state of each point
        state_channel = uwb_fov_mask.channels[3].values
        # ranges of each point 
        range_channel = uwb_fov_mask.channels[0].values
        # dictionary of angles segmented by their range bin
        range_gated_angles = {}
        i = 0
        for i, point in enumerate(points):
            if i > 0:
                if state_channel[i] == MASKABLE:
                    # collect incident angles that belong to a single range bin 
                    bin_index = int(m.floor(range_channel[i]/self.uwb_range_res))
                    if bin_index in range_gated_angles:
                        # add incident angle and angle scalar pair to the current range gate
                        range_gated_angles[bin_index].append((inc_angles[i], angle_scalar[i]) )
                    else:
                        range_gated_angles[bin_index] = [(inc_angles[i], angle_scalar[i])]
        # for each range bin update the ground spectrum 
        for range_bin_index, angle_weight_pairs in range_gated_angles.iteritems():
            self.ground_spectrum.update_ground_spectrum(angle_weight_pairs, range_bin_index, uwb_scan)

    def _evaluate_angle_scalar(self, uwb_fov_mask):
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
                a_scalar = inverse_parabola(azimuth, self.uwb_beam_width)
                angle_scalars.append(a_scalar)

        return angle_scalars

    def _evaluate_incident_angle(self, uwb_fov_mask, s_list):
        '''
            Evaluates the incident angle between the sensing vector and the ground 

            Input: sensing vector, list of numpy vectors 
            the first sensing vector is a zero vector, othe rest of the data vectors
            that orginate from the ground and travel to the radar position 
        '''
        points = uwb_fov_mask.points
        state_channel = uwb_fov_mask.channels[3].values
        i_angles = []
        # to align the angles vector with all the points
        i_angles.append(-1)
        for i, point in enumerate(points):
            if i > 0:
                # if the point is maskable (check state)
                if state_channel[i] == MASKABLE:
                    # note all maskable points should consecative in the list of points
                    sensor_vector = s_list[i]
                    x_loc = point.x
                    y_loc = point.y
                    normal_vector =  self.gplane_map.get_normal(x_loc, y_loc)
                    aspect_angle = angle_between_vectors(normal_vector, sensor_vector)
                    incident_angle = m.fabs(90 - aspect_angle)
                    i_angles.append(incident_angle)
                else:
                    i_angles.append(-1)
        return i_angles

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
                [row_index, col_index] = cartesian_2_index(x_loc, y_loc, self.emap_properties)
                # determine the z projection of the mask point in meters 
                z_proj = self.emap[row_index, col_index]/100.0 
                # check z value of cell is known
                if z_proj == self.emap_error_code:
                    s_list.append(zero_vector)
                else:
                    assert(z_proj != 6)
                    # therefore the point on the ground is 
                    G = np.array([x_loc, y_loc, z_proj])
                    # determine the sensing vector - originates from ground
                    # vector from ground point to sensor origin
                    v_gs = S - G
                    s_list.append(v_gs)

        return s_list

    def _update_uwb_fov_msk_state(self, uwb_fov_mask, max_range):

        '''
            Determines which part of the UWBFOV can be used to determine ground return thresholds 
            and encodes the state value into the state channel of the pointcloud data  
        '''
        ranges = uwb_fov_mask.channels[0].values
        ch_msk = ChannelFloat32()
        ch_msk.name = "mask_state"
        range_thresh = max_range - self.offset
        # ensure range_threshold is a multiple of the range bin size
        # to ensure the mask does not cut off halfway inside a range bin
        bin_index = int(m.floor(range_thresh/self.uwb_range_res))
        # reduce bin index by 1 
        bin_index = bin_index - 1
        range_thresh = self._bin_index_2_range_uper(bin_index)
        # print range_thresh
        i = 0
        for i in xrange(len(ranges)):
            range_v = ranges[i]
            if range_v >= range_thresh:
                ch_msk.values.append(0)
            else:
                ch_msk.values.append(MASKABLE)
        # override the placeholder channel 
        uwb_fov_mask.channels[3] = ch_msk

    def _bin_index(self, range_value):
        '''
            Determine the range bin given a range value 
        '''
        return int(m.floor(range_value/self.uwb_range_res))

    def _bin_index_2_range_uper(self, bin_index):
        '''
            Determine the range value given a bin index
        '''
        return bin_index*self.uwb_range_res + self.uwb_range_res

    def _furthest_bin(self, uwb_fov_mask): 
        '''
            determine the range of the traversable area from the radar sensor  

            input: uwb_fov_maks, datatype sensor_msgs pointcloud 
            encoded with x,y,0 range, azimuth elevation position of the uwb_mask

            returns the range from the sensor where there are no obstacles 
        '''
        max_range = 0
        range_channel = uwb_fov_mask.channels[0].values
        for i, point in enumerate(uwb_fov_mask.points):
            if i > 0:
                # ignore the sensor origin point
                x = point.x
                y = point.y
                range_ = range_channel[i]
                # check if the current cell is traversable 
                [row_index, col_index] = cartesian_2_index(x, y, self.map_properties)
                ti = self.tmap[row_index, col_index]
                if ti == 100 or ti == -1:
                    max_range = range_
                    return max_range
                else:
                    max_range = range_
        return max_range

    def visualise_ground_spectrum(self, uwb_mask_pcd, radar_height=0.7, radar_range_res=0.3):

        '''
            Returns an intensity spectrum assuming the robot is travelling on flat ground.

        '''

        points = uwb_mask_pcd.points
        horizontal_ranges = uwb_mask_pcd.channels[0].values
        ranges = [0]*len(horizontal_ranges)
        # add place holder for the origin point
        angle_mask = [0]
        for i, point in enumerate(points):
            if i > 0:
                angle_mask.append(np.arctan(radar_height/horizontal_ranges[i]))
                # update ranges see page 64 logbook 
                ranges[i] = m.sqrt(radar_height*radar_height + horizontal_ranges[i]*horizontal_ranges[i])

        # for each range bin evaluate the incident angles and azimuth scalar for each incident angle  
        angle_weights_gated = {}
        angle_scalars = self._evaluate_angle_scalar(uwb_mask_pcd)
        for i, angle in enumerate(angle_mask):
            if i > 0:
                bin_index =  int(m.floor(ranges[i]/radar_range_res))
                if bin_index in angle_weights_gated:
                    angle_weights_gated[bin_index].append((angle, angle_scalars[i]))
                else:
                    angle_weights_gated[bin_index] = [(angle, angle_scalars[i])]
        # for each range bin evaluate the expected ground return  
        # if unknown plot as -1 
        intensity_spectrum = {}
        for bin_index, angles_weights in angle_weights_gated.iteritems():
            intensity_spectrum[bin_index] = self.ground_spectrum.expected_ground_return(angles_weights, bin_index)

        intensity_min_max = intensity_spectrum.values()
        intensity_avg = [inten_pair[0] for inten_pair in intensity_min_max]
        intensity_max = [inten_pair[1] for inten_pair in intensity_min_max]
        return [intensity_avg, intensity_max, intensity_spectrum.keys()]











