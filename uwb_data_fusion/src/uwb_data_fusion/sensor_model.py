#!/usr/bin/env python 
import math as m
import numpy as np
from utils_thesis.common import *
from utils_thesis.scaling_functions import inverse_parabola
from scipy import signal
from peakdetect import peakdet


'''
    uwb sensor model class 

    probalility of occupancy is bounded by 0.5-max_prob_occ

    probability of being free is bounded by 0.5-max_prob_free

    returns a uwb pcd, with the range gate probabilities encodeded into it

'''
# GLOBAL CONSTANTS
DEG2RAD = m.pi/180.0

class SensorModel(object):
    def __init__(self, rmin=5.1, rmax=15.1, uwb_beam_width=40, range_res=0.3):

        self.max_prob_occ = 0.8
        self.max_prob_free = 0.8
        self.rmin = rmin
        self.rmax = rmax
        self.uwb_beam_width = uwb_beam_width
        self.range_res = range_res

    def _angle_scalar(self, uwb_pcd):
        '''
            Input: 
                   -uwb_pcd, dtype: pointcloud. Pointcloud representing uwb_mask in global frame with appropiate channel for encoding

            Result: pointcloud, with the angle scalar encoded into a channel 
        '''
        RAD2DEG = 180.0/m.pi
        points = uwb_pcd.points
        azi_channel = uwb_pcd.channels[1].values
        ch_a = uwb_pcd.channels[4].values
        # place holder scalar for the sensor origin point
        ch_a[0] = 0.0
        for i, point in enumerate(points):
            if i > 0:
                # get azimuth position in degrees  
                azimuth = azi_channel[i]*RAD2DEG
                a_scalar = inverse_parabola(azimuth, self.uwb_beam_width)
                assert(a_scalar <= 1)
                assert(a_scalar >= 0.5)
                ch_a[i] = a_scalar

    def _define_region(self, inten_spec, uwb_scan, uwb_pcd):
        '''
            Input:
                    -uwb_pcd, dtype: pointcloud. Pointcloud representing uwb_mask in global frame with appropiate channel for encoding
                    -inten_spec dtype: dictionary. Index by the range_bin index, containing [mean_return, one_std_rtn]
                    -uwb_scan dtype: struct/UWBScan.msg. Contains a list with intensity values for each bin starting from zero 

            Result: pointcloud, with and channel representing detection region (r1 ,r2 or unknown -1) 
        '''
        intensity = uwb_scan.intensity
        state = []  # indexed by range bin
        # iterate the uwb scan and assoicate a state for each range bin index
        for rbin, inten in enumerate(intensity):
            # determine if ground spectrum known for rbin
            if rbin in inten_spec:
                [mean_rtn, one_std_rtn] = inten_spec[rbin]
                if inten >= mean_rtn:
                    state.append(1)
                else:
                    state.append(2)
            else:
                state.append(-1)
        # iterate over pcd, for each point determine what state it belongs to and encode state channel 
        points = uwb_pcd.points
        ranges = uwb_pcd.channels[0].values
        region = uwb_pcd.channels[5].values
        for i, point in enumerate(points):
            # bin index
            if i > 0: 
                bin_index =  int(m.floor(ranges[i]/self.range_res))
                region[i] = state[bin_index]
            else:
                # centre point region is unknown 
                region[i] = -1


    def _power_scalar(self, inten_spec, uwb_scan, uwb_pcd):
        '''
            Input:
                    -uwb_pcd, dtype: pointcloud. Pointcloud representing uwb mask in global frame with appropiate channel for encoding
                                     The region channel shall be set with appropiate values 
                    -inten_spec dtype: dictionary. Index by the range_bin index, containing [mean_return, std_return]
                    -uwb_scan dtype: struct/UWBScan.msg. Contains a list with intensity values for each bin starting from zero 

            Result: pointcloud, with original channels and a channel representing the power scalar value, if power scalar is unknown 
                    then -1
        '''
        # for each point check region
        points = uwb_pcd.points
        region = uwb_pcd.channels[5].values
        power_s = uwb_pcd.channels[6].values
        ranges = uwb_pcd.channels[0].values 
        intensity = uwb_scan.intensity
        for i, point in enumerate(points):
            if i > 0:
                # determine the range bin
                range_bin = int(m.floor(ranges[i]/self.range_res))
                inten = intensity[range_bin]
                [mean_rtn, one_std_rtn] = inten_spec[range_bin]
                #----detection of target----
                if region[i] == 1 and mean_rtn > -1:
                    tmax = self.two_std_i(mean_rtn, one_std_rtn)
                    td = mean_rtn
                    # I > Tmax 
                    if inten > tmax:
                        power_scalar = 1
                    elif inten <= td:
                        # i == td
                        power_scalar = 0.5
                    else:
                        # Td < I < Tmax - scale between 1 and 0.5 
                        d = tmax - td
                        if d < 1:
                            d == 1
                        a = 3/d
                        diff_inten = inten - td
                        power_scalar = self.sigmoid(diff_inten,a)
    
                #---free space detection---  
                if region[i] == 2 and mean_rtn > -1:
                    tmin = self.one_std_down_i(mean_rtn, one_std_rtn)
                    td = mean_rtn
                    # I <= Tmin
                    if inten <= tmin or inten == 0:
                        power_scalar = 1
                    elif inten >= td:
                        # I == td
                        power_scalar = 0.5
                    else:
                        # Tmin < I < Td - scale between 1 and 0.5 
                        d = tmin - td
                        if d < 1:
                            d == 1
                        a = 5/d
                        diff_inten = inten - td
                        power_scalar = self.sigmoid(diff_inten,a)
                        # endif
                if mean_rtn < 0:
                    power_scalar = -1
                # set power scalar 
                power_s[i] = power_scalar
            else:
                power_s[i] = -1

        
    def _power_dispersion_scalar(self, inten_spec, uwb_scan, uwb_pcd):
        '''
            This scalar is only applicable to the R1 region. The lowest value of this scalar is 0.5 

            Input:
                    -uwb_pcd, dtype: pointcloud. Pointcloud representing uwb mask in global frame with appropiate channel for encoding
                                     The region channel shall be set with appropiate values 
                    -inten_spec dtype: dictionary. Index by the range_bin index, containing [mean_return, std_return]
                    -uwb_scan dtype: struct/UWBScan.msg. Contains a list with intensity values for each bin starting from zero 

            Output: pointcloud, with original channels and a channel representing the _power_dispersion_scalar scalar value, 
                    if _power_dispersion_scalar scalar is unknown then -1
        '''
        # YH used a guassian kernal as a scalar 
        kern = np.array( signal.get_window(('gaussian', 4), 9))

        # for the inten_spectrum determine the peaks 
        intensity = list(uwb_scan.intensity)
        [max_pks, values] = peakdet(intensity,2)
        pk_bins = [pair[0] for pair in max_pks]

        points = uwb_pcd.points
        region = uwb_pcd.channels[5].values
        pow_dip = uwb_pcd.channels[7].values
        ranges = uwb_pcd.channels[0].values 

        for i, point in enumerate(points):
            if i > 0:
                if region[i] == 1:
                    range_bin = int(m.floor(ranges[i]/self.range_res))
                    dist2peak = self._distance_to_peak(pk_bins, range_bin)
                    # the kernal specifies weightings based on the distance to peak 
                    # index 0 applies to a pkdist of 4, index 1 appliest to a pkdist of 3 ...
                    kern_index = 4 - dist2peak
                    pow_dip[i] = kern[kern_index]
                else:
                    pow_dip[i] = -1
            else:
                pow_dip[i] = -1

    def _distance_to_peak(self, pk_bins, range_bin):

        max_dis = 4
        for i in xrange(max_dis):
            index_left = range_bin - i
            index_right = range_bin + i
            if index_left in pk_bins or index_right in pk_bins:
                return i
        return max_dis

    def _distance_scalar(self, uwb_pcd):
        '''
            The distance scalar is only applied for R1, [0.5 - 1]

            Input:
                    -uwb_pcd, dtype: pointcloud. Pointcloud representing uwb mask in global frame with appropiate channel for encoding
                                     The region channel shall be set with appropiate values 

            Output: pointcloud, with original channels and a channel representing the distance scalar value, 0.5 - 1
                    if distance scalar is unknown then -1
        '''
        points = uwb_pcd.points
        ranges = uwb_pcd.channels[0].values
        region = uwb_pcd.channels[5].values 
        dist_sc = uwb_pcd.channels[8].values

        for i, point in enumerate(points):
            if i > 0:
                if region[i] == 2:
                    # P = 0.6 at rmax and P = 1 at Rmin
                    # by decreasing c you can lower the probability of free space at rmax, need to adjust offset constant as well if you do
                    c = 2 
                    dist_sc[i] = (self.rmax + self.rmin - ranges[i])/(c*self.rmax) + 0.5
                else:
                    dist_sc[i] = -1
            else:
                dist_sc[i] = -1

    def prob_sen(self, uwb_pcd, inten_spec, uwb_scan):
        '''
            For each point in the UWB mask determine the probability of measurement|H and
            measurement|H' for each cell. 
        '''
        # evaluate scalars
        self._angle_scalar(uwb_pcd)
        self._define_region(inten_spec, uwb_scan, uwb_pcd)
        self._power_scalar(inten_spec, uwb_scan, uwb_pcd)
        self._power_dispersion_scalar(inten_spec, uwb_scan, uwb_pcd)
        self._distance_scalar(uwb_pcd)

        points = uwb_pcd.points
        angle = uwb_pcd.channels[4].values 
        region = uwb_pcd.channels[5].values 
        power = uwb_pcd.channels[6].values 
        disp = uwb_pcd.channels[7].values 
        distance = uwb_pcd.channels[8].values 
        ps_occ = uwb_pcd.channels[9].values
        ps_empty = uwb_pcd.channels[10].values 

        for i, point in enumerate(points):
            if i > 0:
                if region[i] == 1 and angle[i] != -1 and disp[i] != -1  and power[i] != -1:
                    # update ps_occ
                    ps_occ[i] = angle[i]*power[i]*disp[i]*self.max_prob_occ
                    ps_empty[i] = 1 - ps_occ[i]
                    if ps_occ[i] < 0.5:
                        ps_occ[i] = 0.5
                        ps_empty[i] = 0.5
                elif region[i] == 2 and angle[i] != -1 and power[i] != -1 and distance[i] != -1:
                    # update ps_empty
                    ps_empty[i] = angle[i]*power[i]*distance[i]*self.max_prob_free
                    ps_occ[i] = 1 - ps_empty[i]
                    if ps_empty[i] < 0.5:
                        ps_occ[i] = 0.5
                        ps_empty[i] = 0.5
                else: 
                    # unknown region 
                    ps_occ[i] = -1
                    ps_empty[i] = -1
            else:
                ps_occ[i] = -1
                ps_empty[i] = -1

    @staticmethod
    def two_std_i(mean, one_std_rtn):
        return (one_std_rtn - mean)*2 + mean

    @staticmethod
    def one_std_down_i(mean, one_std_rtn):
        tmin = mean - (one_std_rtn - mean)
        if tmin < 0:
            tmin = 0
        return tmin

    @staticmethod
    def sigmoid(diff_inten, a):
        return 1/ (1.0 + m.exp(-a*diff_inten))