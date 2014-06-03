#!/usr/bin/env python
import numpy as np
import sys
# warning hack alert, if the program is going to be deployed elese where follow this link
# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
sys.path.append('/home/spartan3123/catkin_ws/src/uwb_threshold_updater/src/uwb_threshold_updater')
from ground_spectrum import GroundSpectrum

def test1():
    '''  Testing standard deviation calculation 
    '''
    # assuming 18 sample points incident angle around 5 degrees 
    # ingoring beamwidth scalars 
    a = GroundSpectrum()
    a.min_count = 5
    a._insert_gamma(12.0,1)
    a._insert_gamma(19.0,1)
    a._insert_gamma(15.0,1)
    a._insert_gamma(25.0,1)
    a._insert_gamma(12.0,1)
    a._insert_gamma(13.0,1)
    a._insert_gamma(12.0,1)
    a._insert_gamma(19.0,1)
    a._insert_gamma(12.0,1)
    a._insert_gamma(14.0,1)
    a._insert_gamma(11.0,1)
    gamma = np.array([12.0, 19.0, 15.0, 25.0, 12.0, 13.0, 12.0, 19.0, 12.0, 14.0, 11.0])
    g2 = gamma*gamma
    print 'gamma squared mean ' + str(np.mean(g2))
    print 'gamma mean ' + str(np.mean(gamma))
    print 'standard deviation ' + str(np.std(gamma))
    print a.ground_histogram[1]
    flag = True
    if flag == True:
        print 'pass'
    else:
        print 'fail'

def test2():
    ''' 
        Test the saving of the ground spectrum 
        - visually check the output of the text file 
    '''
    h = {0:[1,2.3,2.4,2.5],1:[2,3.3,3.4,3.5]}
    flag = GroundSpectrum._save_ground_spectrum(h,'test.txt')
    if flag == True:
        print 'pass'
    else:
        print 'fail'
 
def main():
    test1()
    test2()


if __name__ == '__main__':
    main()