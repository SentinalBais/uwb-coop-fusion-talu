import numpy as np
import math as m

'''
    Cell data structure for elevation mapping 
'''

class Cell(object):
    def __init__(self, bin_min_capacity=3, bin_size=0.05):

        self.z_value_min = 9
        self.z_value_max = -9
        self.z_value_avg = 9
        # whether the cell has a known elevation value, requires elevation histogram to have a bin
        # with a freqency greater than the bin min capacity 
        self.known = False
        # bin min capacity 
        self.bin_min_capacity = bin_min_capacity
        # bin size (meters) - discretisation of the z values  
        self.bin_size = bin_size
        self.count = 0
        self.points = []
        self.zHist = {}

    def _index_to_z(self, index):
        '''
            Index should refer to the middle of the bin
            therefore bin 0, which may be 0-5cm will have a z value of 2.5cm 
        '''
        return index*self.bin_size + self.bin_size/2

    def _eval_stats(self, point, index):
        '''
            Checks if bin capacity is greater than equal to the bin min capacity 
            if so update the z value statisics. Given a point and the its bin index
        '''
        # determine if the capacity is greater than the min_bin capacity 
        if self.zHist[index] >= self.bin_min_capacity:
            self.known = True
            z_value = point.z                
            if z_value > self.z_value_max:
                self.z_value_max = z_value
            if z_value < self.z_value_min:
                self.z_value_min = z_value

    def addPoint(self, point):
        # add z value to histogram 
        index = int(point.z/self.bin_size)
        # check if z value is in histogram 
        if index in self.zHist:
            # increment bin counter
            self.zHist[index] = self.zHist[index] + 1
            # checks if bin capacity is valid if so, determine if new bin results in new z_max/z_min
            self._eval_stats(point, index)
        else:
            # add new bin and increment counter 
            self.zHist[index] = 1
            # checks if bin capacity is valid if so, determine if new bin results in new z_max/z_min
            self._eval_stats(point, index)
        # average value
        if self.count == 0:
            self.z_value_avg = point.z
        else:
            '''
            (derivation)
            average of z values 
            average_i = sum/N
            sum =  average*N
            average_i+1 = (sum + z)/(N+1)
            average_i+1 = (average_i*N + z)/(N+1)
            '''
            self.z_value_avg = ((self.z_value_avg*self.count + point.z)/(self.count + 1))
        # increase count
        self.count = self.count + 1
            
    def addPoints(self, points):
        ''' 
            Add points to the cell and update
            average, min, max, count, known(bool)
        '''
        for point in points:
            self.addPoint(point)

    def set_z(self, z_value):
        '''
            Mannualy set the z value of the cell. Once set the cell
            is also vaild
        '''
        self.z_value_avg = z_value
        self.z_value_max = z_value
        self.z_value_min = z_value
        self.known = True

    def __str__(self):
        '''
            Print overloading
        '''
        print_str = str(self.z_value_min)
        return print_str
