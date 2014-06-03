#!/usr/bin/env python
import sys
sys.path.append('/home/spartan3123/catkin_ws/src/uwb_data_fusion/src/uwb_data_fusion')
from sensor_model import SensorModel


def test1():
    ''' testing sigmoid function 
        c = 3 if I > Td
    '''
    print SensorModel.sigmoid(0,5,5)
    flag = True
    if flag == True:
        print 'pass'
    else:
        print 'fail'


 
def main():
    test1()


if __name__ == '__main__':
    main()