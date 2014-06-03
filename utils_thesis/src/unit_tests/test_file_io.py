#!/usr/bin/env python

import sys
# warning hack alert, if the program is going to be deployed elese where follow this link
# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
sys.path.append('/home/spartan3123/catkin_ws/src/utils_thesis/src/utils_thesis')
from file_io import np_2Darray_to_file
from file_io import load_np_2Darray_from_file
import matplotlib.pyplot as plt
import numpy as np


def test1():
    ''' test that the traversability map is printed to a file in the correct format row_major format 
        visual check that the file write is correct 
    '''
    a = np.zeros(shape=(100,100), dtype=int)
    # row 1 is 100
    # col 1 is -1 
    # x axis is _|_ to rows, rows index x axis
    for i, row in enumerate(a):
        for j, element in enumerate(row):
            if i == 1:
                a[i][j] = 100
            if j == 1:
                a[i][j] = 50
    np_2Darray_to_file(a, 'test.txt')
    plt.imshow(a)
    plt.show()
    print 'Pass'

def test2():
    ''' test loading of 2D array from a textfile in row_major order 
    '''
    tmap = load_np_2Darray_from_file(fname="test.txt")
    tmap = tmap.astype(int)
    # print tmap.shape
    plt.imshow(tmap)
    plt.show()
    print 'Pass'

def main():
    print "Unit Testing"
    test1()
    test2()

if __name__ == '__main__':
    main()