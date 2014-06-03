#!/usr/bin/env python
import sys
sys.path.append('/home/spartan3123/catkin_ws/src/traversability_analysis/src/traversability_index/functions')
sys.path.append('/home/spartan3123/catkin_ws/src/traversability_analysis/src/data_structures')
from plane_fit import plane_fit
from plane_fit import slope_plane
from point import Point
import numpy as np
import math as m

def test_3():
    x = [-27.2, -27.2, -27.2, -27.0, -27.0, -27.0, -26.79, -26.79, -26.79]
    y = [-17.2, -17.0, -16.79, -17.20, -17.0, -16.79, -17.20, -17.0, -16.79]
    z = [0.33003, 0.351459, 0.35531, 0.3388, 0.34575, 0.3414, 0.3349, 0.342, 0.329]
    points = []
    print len(x)
    print len(y)
    print len(z)
    for i in xrange(len(x)):
        point = Point(x[i], y[i], z[i])
        points.append(point)
    P = plane_fit(points)
    N = P[0:3]
    print N
    slope = slope_plane(N)
    print slope
    if slope > m.pi/2:
        slope = slope - m.pi
        print slope

def test_2():

    x = [-27.2, -27.2, -27.2, -27.0, -27.0, -27.0, -26.799999999999997, -26.799999999999997, -26.799999999999997]
    y = [-17.200000000000003, -17.0, -16.799999999999997, -17.200000000000003, -17.0, -16.799999999999997, -17.200000000000003, -17.0, -16.799999999999997]
    z = [0.3300317666, 0.3514598337, 0.3553192196, 0.3388973941, 0.3457568605, 0.3414123164, 0.3349728212, 0.3428124056, 0.3295544401]
    points = []
    print len(x)
    print len(y)
    print len(z)
    for i in xrange(len(x)):
        point = Point(x[i], y[i], z[i])
        points.append(point)
    P = plane_fit(points)
    N = P[0:3]
    print N
    slope = slope_plane(N)
    print slope
    if slope > m.pi/2:
        slope = slope - m.pi
        print slope

def test_1():
    # random data test
    A = 2
    B = 3
    C = 2.5
    D = -1
    x = 10*np.random.randn(9)
    y = 10*np.random.randn(9)
    z = -(A*x + B*y + D)/C + 0.1*np.random.randn(len(x))
    points = []
    for i in xrange(len(x)):
    	point = Point(x[i], y[i], z[i])
    	points.append(point)
    P = plane_fit(points)
    P = 2*P/P[0]
    print P
    print 'pass'

def main():
	test_1()
	print "finished"

if __name__ == '__main__':
    main()