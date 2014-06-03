#!/usr/bin/env python 
import math as m

'''
    Object store statistics on the ground return
    the object histograms statistics for each range bin.

    The range bins index is evaluated externally by the following equation:
    range_bin_index = int(m.floor(range/uwb_range_res))
'''

class GroundSpectrum(object):
    def __init__(self, ground_histogram={}):
        
        # a dictionary index by range bin
        # count of gamma values 
        # mean(gamma) (update incrementally)
        # mean(gamma*gamma) (updated incrementally)
        # standard deviation of gamma (updated incrementally)
        # [c, u, u2, sigma(u)]
        # if c is too low, the mean could be interpolated from previous values (this is unlikely to happen)
        # especially if an intial training set is provided  
        '''
            (derivation)
            average of z values 
            average_i = sum/N
            sum =  average*N
            average_i+1 = (sum + z)/(N+1)
            average_i+1 = (average_i*N + z)/(N+1)
        '''
        self.ground_histogram = ground_histogram
        # min number of measurements for range bin 
        self.min_count = 20
        # for statistical analysis of the distribution
        self.gamma_list = []


    def update_ground_spectrum(self, weighted_angles, range_bin_index , uwb_scan):
    	'''
    		Updates the ground spectrum. For range bin, evaluate the average gamma 

    		Input: weighted_angles, a list of tuples containing an incident angle and a weight

    	'''
        DEG2RAD = m.pi/180.0
        # Signal Amplitude for range bin
        intensity = uwb_scan.intensity[range_bin_index]*1.0 # float conversion
        # evaluate gamma 
        sum_inc_angles = 0
        for angle_weight in weighted_angles:
            angle = angle_weight[0]
            weight = angle_weight[1]
            sum_inc_angles = sum_inc_angles + m.sin(DEG2RAD*angle)*weight
            assert(m.sin(DEG2RAD*angle)*weight > 0)
        assert(sum_inc_angles > 0)
        if sum_inc_angles > 0:
            gamma = intensity/sum_inc_angles
            self._insert_gamma(gamma, range_bin_index)

    def _insert_gamma(self, gamma, range_bin_index):
        '''
            inserts gamma into the gamma statistical accumerlator 
        '''
        if range_bin_index == 20:
            self.gamma_list.append(gamma)

        if range_bin_index in self.ground_histogram:
            [count, mean_gamma, mean_gamma_sqrd, std_gamma] = self.ground_histogram[range_bin_index]
            # update state for range bin
            count, mean_gamma, mean_gamma_sqrd, std_gamma = (count + 1,
                                                             (mean_gamma*count + gamma)/(count + 1),
                                                             (mean_gamma_sqrd*count + gamma*gamma)/(count + 1),
                                                             m.sqrt(mean_gamma_sqrd - mean_gamma*mean_gamma))

            self.ground_histogram[range_bin_index] = [count, mean_gamma, mean_gamma_sqrd, std_gamma]

        else:
            self.ground_histogram[range_bin_index] = [1, gamma, gamma*gamma, 0]

    @staticmethod
    def eval_incident_angle_sum(angle_weights):
        '''
            input: angle_weights, a list of tupples containing an incident angle and an assoicated weight
                   where the angles are in radians and the weight is a number between [0-1]
        '''
        incident_angle_sum = 0
        for i, angle_weight in enumerate(angle_weights):
            angle, weight = angle_weight[0], angle_weight[1]
            incident_angle_sum = incident_angle_sum + m.sin(angle)*weight
            assert(m.sin(angle_weight[0])*angle_weight[1] > 0)
        return incident_angle_sum

    def expected_ground_return(self, angle_weights, range_bin_index):
        '''
            Estimate the ground return given the estimated incident angle sum

            input: angle_weights, a list of tupples containing an incident angle and an assoicated weight
                   where the angles are in radians and the weight is a number between [0-1]
        '''
        
        incident_angle_sum = self.eval_incident_angle_sum(angle_weights)
        # does the current range bin have a gamma statistics 
        if range_bin_index in self.ground_histogram:
            [count, mean_gamma, mean_gamma_sqrd, std_gamma] = self.ground_histogram[range_bin_index]
        else:
            return [-1, -1]

        if count > self.min_count:
            ground_return = mean_gamma*incident_angle_sum
            ground_return_max = (mean_gamma + std_gamma)*incident_angle_sum
            return [ground_return, ground_return_max]
        else:
            return [-1, -1]

    def save_ground_spectrum(self, fname="spectrum.txt"):
        '''
            Class wrapper for saving ground spectrum 
        '''
        self.__save_ground_spectrum(self.ground_histogram, fname)

    @staticmethod
    def _save_ground_spectrum(ground_histogram, fname):
        '''
            Save the ground spectrum histogram as a file 
            Values

            The ground spectrum histogram is indexed by the range bin index.
            Each bin will contain a list in the following format [count,mean_gamma,mean_gamma_sqrd,std_gamma] dtype:double

            Each line shall have the following format 
            count,mean_gamma,mean_gamma_sqrd,std_gamma
               - where the data, type of items is double
               - unknown value shall be indicated with a -1 

            Each histogram entry shall be on seperate lines
            count,mean_gamma,mean_gamma_sqrd,std_gamma ( bin index 1)
            count,mean_gamma,mean_gamma_sqrd,std_gamma ( bin index 2)

            returns True if finished 
        '''
        # iterate through the ground histogram an collect a list of lists which will be output to file 
        data = []
        for key, row in ground_histogram.iteritems():
            data.append(row)

        # file io 
        with open(fname, 'w') as f:
            # load data into textfile in row major order 
            for row in data:
                row_str = str(row)
                # removes trailing or leading braces
                row_str = row_str.strip('[]')
                # make the items in the row commma seperated
                row_str = ''.join(row_str.split())
                # add a new line character 
                row_str = row_str + '\n'
                f.write(row_str)
        return True





