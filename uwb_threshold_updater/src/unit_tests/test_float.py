#!/usr/bin/env python
import sys
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
sys.path.append('/home/spartan3123/catkin_ws/src/uwb_threshold_updater/src')

def test1():
    ''' testing uwb_threshold_updater static functions 
    '''
    a = 6.9
    ch_range = ChannelFloat32()
    ch_range.name = "range"
    ch_range.values.append() 

    if flag == True:
        print 'pass'
    else:
        print 'fail'


 
def main():
    test1()


if __name__ == '__main__':
    main()