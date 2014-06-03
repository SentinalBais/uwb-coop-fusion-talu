import numpy as np
import rospy
from cell import Cell
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
import math as m


class ElevationMap(object):
    def __init__(self, delta_x, delta_y, cell_size, z_min, z_max, bin_min_cap=2):
        self.z_min = z_min
        self.z_max = z_max
        self.g_z_max = -100
        self.g_z_min = 100
        self.delta_x = delta_x
        self.delta_y = delta_y
        self.cell_size = cell_size
        self.bin_min_cap = bin_min_cap
        self.num_points = 0
        # CONSTANTS
        EDGE_PADDING = 20
        # intilise the elevation map
        # x axis || collumns, each row is a new x value, x_index = row_index
        # y axis || rows, each col is a new y value  y_index = col_index
        ''' (elevation map coordinate frame)
        (x axis)
        ^
        |
        |
        *_________> (y axis)
        '''
        self.num_rows = int(delta_x/cell_size) + EDGE_PADDING
        self.num_cols = int(delta_y/cell_size) + EDGE_PADDING
        # coordinate of origin
        self.origin_x = int(round(delta_x/cell_size/2))
        self.origin_y = int(round(delta_y/cell_size/2))

        # initilise 2D list with empty cell objects
        cell_map = [[Cell(bin_min_capacity=bin_min_cap) for i in xrange(self.num_cols)] for j in xrange(self.num_rows)]
        # create numpy 2D array of cells  
        self.map = np.array(cell_map)

    def addPoint(self, point):
        ''' adds a point into the elevation map
            The z value for a point must be between z_min and z_max 
            otherwise the point will not be added into the elevation map
        '''
        # determine the index
        [x_index, y_index] = self._index(point)
        # add point to cell
        if point.z < self.z_max and point.z > self.z_min:
            self.map[x_index, y_index].addPoint(point)
            self.num_points = self.num_points + 1
            # determine the global z_min and z_max
            if point.z > self.g_z_max:
                self.g_z_max = point.z
            if point.z < self.g_z_min:
                self.g_z_min = point.z

    def addPoints(self, points):
        ''' adds a vector of points into the elevation map
            The z value for a point must be between z_min and z_max 
            otherwise the point will not be added into the elevation map
        '''
        for point in points:
            self.addPoint(point)

    def _index(self, point):
        ''' 
            evaluates the index of point
        '''
        # zero_zero is the centre of the Elevation Map
        row_index = int(round(point.x/self.cell_size)) + self.origin_x
        col_index = int(round(point.y/self.cell_size)) + self.origin_y
        return [row_index, col_index]

    def index_to_cart(self, row_index, col_index):
        '''
            Given a cells, row index and collumn index, determine its location relative to the map
            coordinate frame. The location of a cell is defined as its centre
        '''
        # where origin_x/origin_y is the coordinate of the origin 
        x_value = row_index*self.cell_size - self.origin_x*self.cell_size
        y_value = col_index*self.cell_size - self.origin_y*self.cell_size
        return [x_value, y_value]
 
    def getZ_max(self):
        ''' returns a numpy array containing the max z value for each cell
            if cell is unknown then the z value will be z_max + 1 
        ''' 
        z_max = np.zeros(shape=(self.map.shape))
        for x_index, row in enumerate(self.map):
            for y_index, cell in enumerate(row):
                # check if cell is valid
                if cell.known == True:
                    z_max[x_index, y_index] = cell.z_value_max
                else:
                    z_max[x_index, y_index] = self.z_max + 1
        return z_max

    def getZ_min(self):
        ''' returns a numpy array containing the min z value for each cell
            if cell is unknown then the z value will be z_max + 1 
        '''
        z_min = np.zeros(shape=(self.map.shape))
        for x_index, row in enumerate(self.map):
            for y_index, cell in enumerate(row):
                # check if cell is valid
                if cell.known == True:
                    z_min[x_index, y_index] = cell.z_value_min
                else:
                    z_min[x_index, y_index] = self.z_max + 1
        return z_min


    def getZ_avg(self):
        ''' returns a numpy array containing the avgerage z value for each cell
            If cell is unknown then the z value will be z_max + 1 
        '''

        z_avg = np.zeros(shape=(self.map.shape))
        for x_index, row in enumerate(self.map):
            for y_index, cell in enumerate(row):
                # check if cell is valid
                if cell.known == True:
                    z_avg[x_index, y_index] = cell.z_avg
                else:
                    z_avg[x_index, y_index] = self.z_max + 1
        return z_avg


    def get_pointcloud(self, meta_data=[]):
        ''' Publish a pointcloud representation of the elevation map
            If the elevation of a cell is unknown then a cell will have
            no pointclouds
            -meta_data is a 2D array of structs containing the metrics to evaluate the ti index
        '''
        z_min_cloud = PointCloud()
        z_avg_cloud = PointCloud()
        z_max_cloud = PointCloud()
        # channels for meta data if given
        # meta data will be encoded into the z_max cloud
        channel_a = ChannelFloat32()
        channel_a.name = "slope"    
        channel_b= ChannelFloat32()
        channel_b.name = "roughness"    

        # iterate through each cell in the elevation map
        # determine x,y coordinates. Map indexes refference the centre of the cells
        for row_index, row in enumerate(self.map):
            x_index = row_index
            for col_index, cell in enumerate(row):
                y_index = col_index
                if cell.known == True:
                    # based on index i and j allocated x,y of point
                    x_value = (x_index - self.origin_x)*self.cell_size
                    y_value = (y_index - self.origin_y)*self.cell_size 
                    z_min = cell.z_value_min
                    z_avg = cell.z_value_avg
                    z_max = cell.z_value_max
                    # add meta data into intensity channels 
                    if len(meta_data) > 0:
                        if meta_data[row_index, col_index].known == True:  
                            channel_a.values.append(abs(meta_data[row_index, col_index].slope)*180/m.pi)
                            channel_b.values.append(meta_data[row_index, col_index].roughness_metric)
                        else:
                            channel_a.values.append(-1)
                            channel_b.values.append(-1)
                    else:
                        channel_a.values.append(-1)
                        channel_b.values.append(-1)
                    # generate point where the z_value is for each cell
                    self._gen_point(x_value, y_value, z_min, z_min_cloud)
                    self._gen_point(x_value, y_value, z_avg, z_avg_cloud)
                    self._gen_point(x_value, y_value, z_max, z_max_cloud)

        z_min_cloud.channels.append(channel_a)
        z_min_cloud.header.frame_id = "map"
        z_min_cloud.header.stamp = rospy.Time.now()

        z_avg_cloud.header.frame_id = "map"
        z_avg_cloud.header.stamp = rospy.Time.now()
        # add meta data to the z_max cloud 
        z_max_cloud.channels.append(channel_a)
        z_max_cloud.header.frame_id = "map"
        z_max_cloud.header.stamp = rospy.Time.now()
        return [z_min_cloud, z_avg_cloud, z_max_cloud]

    def interplotate(self, window_size=3, k=-1):
        '''
            Interpolates the elevation values if a cell has k known neighbours 
            The z_max value is interpolated. Only Unkown cells are interploated 
        '''
        ''' (elevation map coordinate frame)
        (x axis)
        ^
        |
        |
        *_________> (y axis)
        '''
        if k == -1:
            k = int(round(window_size*window_size*0.55))
        else:
            if k > window_size*window_size - 1:
                k = window_size*window_size - 1

        # x_index = row_index
        # y_index = col_index
        # offset ensures window does not go over boundaires of elevation map 
        offset = (window_size - 1)/2
        for row_index in xrange(0 + offset, self.map.shape[0] - offset):
            for col_index in xrange(0 + offset, self.map.shape[1] - offset):
                cell = self.map[row_index, col_index]
                # only interploate unkown cells
                if cell.known == False:
                    n_indexs = self._get_index_window(row_index, col_index, window_size)
                    count = 0
                    z_values = []
                    # get the z values of the neighbours if cell is known
                    for index in n_indexs:
                        x = index[0]
                        y = index[1]
                        if self.map[x, y].known == True:
                            count = count + 1 
                            z_values.append(self.map[x, y].z_value_max)
                    if count >= k:
                        # interplotate the z value 
                        cell.set_z(sum(z_values)/len(z_values))

    def _get_index_window(self, centre_row_index, centre_col_index, window_size):
        window = []
        window_side_len = (window_size - 1)/2
        start_row_i = centre_row_index - window_side_len
        end_row_i = centre_row_index + window_side_len

        start_col_index = centre_col_index - window_side_len
        end_col_index = centre_col_index + window_side_len

        for i in xrange(start_row_i, end_row_i + 1):
            for j in xrange(start_col_index, end_col_index + 1):
                window.append([i, j])
        return window

            

    def _gen_point(self, x, y, z, pointcloud):
        '''
            Generate point to represent an elevation at x, y and append point to 
            pointcloud refference
        '''
        # A box that is the size of z_inc will represent the elevation 
        # of a cell, therefore the 1/2 the width of a box must be added(+z is down) to the 
        # z value, so the top surface of the box represents the elevation of the cell
        z = z + self.cell_size/2.0
        point_msg = Point32()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = z
        pointcloud.points.append(point_msg)

    def _gen_points(self, x, y, z_start, z_end, z_inc , pointcloud):
        ''' Generate points to represent the elevation at, x,y and Appends
           these points into a pointcloud
        '''
        # z values will be relative to the map refference frame.
        # As per engineering conventions the z axis is directed towards the ground
        # therefore the z_start value will be positive and the z_end value will be 
        # smaller than the z_start value. zinc must therefore be negated
        for z_value in np.arange(z_start, z_end, -z_inc):
            z_value = z_value - z_inc/2
            self._gen_point(x, y, z_value, pointcloud)
        
    def __str__(self):
        line = [str(self.x), str(self.y), str(self.z)]
        print_str = ' '.join(line)
        return print_str

