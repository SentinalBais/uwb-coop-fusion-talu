import rosbag
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32
import numpy as np
from rospy import Time
import tf
import math as m
from point import Point
from uwb_radar_msgs.msg import UWBScan


''' Thesis Utilities: Converts data sets into rosbag file
    Thivanka Aluwihare 
'''

def main():
    filename_radar = "datasets/clear/uwb.txt"
    filename_nav = "datasets/clear/nav.csv"
    file_name_csv = "datasets/clear/sick.csv"
    file_name_pcd = "datasets/clear/pcd/point_cloud.pcd"
    bag_file_name = "clear.bag"
    # parse the lidar data and determine the local origin  
    [x, y, z, origin] = read_lidar_csv(file_name_csv)
    points = []
    for i , x_val in enumerate(x):
        points.append(Point(x[i], y[i], z[i]))
    pcd_io(file_name_pcd, points)
    [uwb_msgs, uwbpcd] = read_uwb_csv(filename_radar)
    tf_msgs = read_nav_csv(filename_nav, origin)
    pcd_msg = pointcloud_msg(points)
    write_bagfile(bag_file_name, uwb_msgs, tf_msgs, pcd_msg, uwbpcd)


def pcd_io(fname,points):
    ''' Writes a vector of points into a PCL pcd file 
    '''
    num_points = len(points)
    intensity = '255'
    NEWLINE = "\n"
    with open(fname, 'w') as w:
        w.write("# .PCD v.7 - Point Cloud Data file format" + NEWLINE)
        w.write("VERSION .7" + NEWLINE)
        w.write("FIELDS x y z intensity" + NEWLINE)
        w.write("SIZE 4 4 4 4" + NEWLINE)
        w.write("TYPE F F F F" + NEWLINE)
        w.write("COUNT 1 1 1 1" + NEWLINE)
        w.write("WIDTH " + str(num_points) + NEWLINE)
        w.write("HEIGHT 1" + NEWLINE)
        w.write("VIEWPOINT 0 0 0 1 0 0 0" + NEWLINE)
        w.write("POINTS " + str(num_points) + NEWLINE)
        w.write("DATA ascii" + NEWLINE)
        # start writing in points (x,y,z)
        for point in points:
            line = [str(point.x), str(point.y), str(point.z), intensity, NEWLINE]
            line_str = ' '.join(line)
            w.write(line_str)

    print "Finished PCD IO"

def read_nav_csv(fname, origin):
    ''' Reads the nav.csv , converts into a local coordinate frame and returns tf msgs '''
    msgs = []
    DEG2RAD = m.pi/180.0  
    with open(fname, 'r') as f:
        # example data 
        # 1354679075.295000000,6248548.85357,332949.839026,-41.4911136152,-0.00740605127066,-0.0505019649863,1.95254564285
        # a Nx7 matrix containing pose [time N E D R P Y] UTM cartesian coordinate system  
        for i, line in enumerate(f):
            words = line.split(',')
            time = words[0]
            [secs, nsecs]  = map(int, time.split('.'))
            x = float(words[1])
            y = float(words[2])
            z = float(words[3])
            R = float(words[4])
            P = float(words[5])
            Y = float(words[6])
            # convert to local coordinate frame, located at the origin 
            x = x - origin[0]
            y = y - origin[1]
            z = z - origin[2]
            # create TF msg 
            tf_msg = tfMessage()
            geo_msg = TransformStamped()
            geo_msg.header.stamp = Time(secs,nsecs) 
            geo_msg.header.seq = i
            geo_msg.header.frame_id = "map"
            geo_msg.child_frame_id = "body"
            geo_msg.transform.translation.x = x
            geo_msg.transform.translation.y = y
            geo_msg.transform.translation.z = z
            angles = tf.transformations.quaternion_from_euler(R,P,Y) 
            geo_msg.transform.rotation.x = angles[0]
            geo_msg.transform.rotation.y = angles[1]
            geo_msg.transform.rotation.z = angles[2]
            geo_msg.transform.rotation.w = angles[3]
            # rviz frame 
            geo_msg2 = TransformStamped()
            geo_msg2.header.stamp = Time(secs,nsecs) 
            geo_msg2.header.seq = i
            geo_msg2.header.frame_id = "map_rviz"
            geo_msg2.child_frame_id = "map"
            geo_msg2.transform.translation.x = 0
            geo_msg2.transform.translation.y = 0
            geo_msg2.transform.translation.z = 0
            angles = tf.transformations.quaternion_from_euler(DEG2RAD*180, 0, 0) # ax, ay, az 
            geo_msg2.transform.rotation.x = angles[0]
            geo_msg2.transform.rotation.y = angles[1]
            geo_msg2.transform.rotation.z = angles[2]
            geo_msg2.transform.rotation.w = angles[3]
            # push all transformations 
            tf_msg.transforms.append(geo_msg)
            tf_msg.transforms.append(geo_msg2)
            msgs.append(tf_msg)

    return msgs

def read_uwb_csv(fname):
    ''' Reads a UWB data and creats a list of msgs that will be written into a bag file 
    '''
    UWB_DATA_START = 14
    UWB_TX_ATTEN = 0            # dB 
    UWB_RX_ATTEN = 0            # dB
    UWB_RANG_RES = 0.3048       # m
    RANGE_BINS = 256
    FRAME_ID = "radar"
    intensities = []
    msgs = []
    seq_i = 0
    msgs_pointclouds = []
    with open(fname, 'r') as f:
        for i, line in enumerate(f):
            if i > UWB_DATA_START:
                words = line.split(' ')
                time = words[0]                                 # extract the time
                [secs, msecs]  = map(int, time.split('.'))
                nsecs = int(msecs*10**6)                        # converting from mili seconds to nano seconds
                del words[0]                                    # remove the time so words are only intensities 
                words.pop()                                     # remove the new line character 
                intensities = map(int, words)                   # converts each word into an int 
                radar_scan = UWBScan()
                radar_scan.header.stamp = Time(secs,nsecs) 
                radar_scan.header.frame_id = FRAME_ID
                radar_scan.header.seq = seq_i
                radar_scan.range_bins = RANGE_BINS
                radar_scan.range_resolution = UWB_RANG_RES
                radar_scan.tx_attenuation = UWB_TX_ATTEN
                radar_scan.rx_attenuation = UWB_RX_ATTEN
                radar_scan.intensity = intensities
                msgs.append(radar_scan)
                seq_i = seq_i + 1
                if i % 2 == 0:
                    pcd = uwb_vis(radar_scan)
                    msgs_pointclouds.append(pcd)
    return [msgs, msgs_pointclouds]

def uwb_vis(radar_scan):
    ''' Represents the UWB radar scan as a point cloud 
    '''
    # each amplitude value is associated with a range bin 
    MIN_RANGE = 1 # radar_scan.range_resolution/2
    MAX_RANGE = 15 # len(radar_scan.intensity)*radar_scan.range_resolution
    INC = 0.1   # meters   
    INC_AZ = 1 # degrees 
    AZ_LEFT = -22
    AZ_RIGHT = 22
    msg = PointCloud()
    inten = ChannelFloat32()
    inten.name = "intensity"
    points = []
    for _range in np.arange(MIN_RANGE, MAX_RANGE, INC):
        for az in np.arange(AZ_LEFT, AZ_RIGHT, INC_AZ):
            # create a point
            point = Point(cart=False, range_m=_range, az=az)
            points.append(point)
            point_msg = Point32()
            point_msg.x = point.x
            point_msg.y = point.y
            point_msg.z = point.z
            # find the appropiate range bin 
            bin = int(point.range/radar_scan.range_resolution)
            # get the correct intensity value 
            inten.values.append(radar_scan.intensity[bin]*1.0)
            #inten.values.append(30)
            msg.points.append(point_msg)
    # pcd_io("test.pcd", points)
    msg.channels.append(inten)
    msg.header.frame_id = "body"
    secs = radar_scan.header.stamp.secs 
    nsecs = radar_scan.header.stamp.nsecs
    msg.header.stamp = Time(secs,nsecs)
    return msg

def write_bagfile(data_set_name, uwb_msgs, tf_msgs, point_cloud_msg , uwb_pcd):
    ''' Write all ROS msgs, into the bagfile. 
    '''
    bag = rosbag.Bag(data_set_name, 'w')
    # write the tf msgs into the bagfile
    for i, msg in enumerate(tf_msgs):
        bag.write('/tf', msg, msg.transforms[0].header.stamp )
        if i == 0:
            bag.write('laser', point_cloud_msg, msg.transforms[0].header.stamp )
    # write the uwb msgs into the bagfile 
    for msg in uwb_pcd:
        bag.write('radar_scan_viz', msg, msg.header.stamp)

    for msg in uwb_msgs:
        bag.write('radar_scan', msg, msg.header.stamp)
    bag.close()
    print "Finished Bagfile IO"

def pointcloud_msg(points):

    msg = PointCloud()
    inten = ChannelFloat32()
    inten.name = "intensity"
    for point in points:
        point_msg = Point32()
        point_msg.x = point.x
        point_msg.y = point.y
        point_msg.z = point.z
        inten.values.append(255)
        msg.points.append(point)
    # write header
    msg.channels.append(inten)
    msg.header.frame_id = "map"
    return msg

def read_lidar_csv(fname):
    '''Reads a lidar.csv file and returns a vector [x,y,z] relative to a corrected coordinate
       system which is centred in the middle of pointcloud (in the x and y plane)
       the orgin (0,0,0) should be where the centroid of the pointcloud is
     '''
    xl = []
    yl = []
    zl = []
  # reading the csv file recorded from the mantis, using the ACFR c++ libaries
    with open(fname, 'r') as f:
        for line in f:
            words = line.split(',')
            xl.append(words[4])
            yl.append(words[5])
            zl.append(words[6])
    # determine the origin position
    # x,y,z are in UTM cartesian coordinate system with the orgin far away 
    x = np.array(xl, dtype=np.float64)
    y = np.array(yl, dtype=np.float64)
    z = np.array(zl, dtype=np.float64)


    max_x = np.max(x)
    min_x = np.min(x)
    max_y = np.max(y)
    min_y = np.min(y)

    max_z = np.max(z)
    min_z = np.min(z)

    x_origin = int( (max_x - min_x)/2 + min_x )
    y_origin = int( (max_y - min_y)/2 + min_y )
    z_origin = int( (max_z - min_z)/2 + min_z )

    # convert points so they are relative to a new coordinate system 
    x = x - x_origin;
    y = y - y_origin;
    z = z - z_origin;

    origin = [x_origin, y_origin, z_origin]

    return [x, y, z, origin]


if __name__ == '__main__':
    main()


