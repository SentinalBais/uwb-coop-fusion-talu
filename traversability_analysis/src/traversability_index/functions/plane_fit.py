#!/usr/bin/env python
import numpy as np
import math as m

def plane_fit(points=[], xl=[], yl=[], zl=[]):
    '''
        Finds the equation of a plane that best fits the given points
    '''
    if len(xl) == 0 and len(yl) == 0 and len(zl) == 0:
        [xl, yl, zl] = split_points(points)


    x = np.array(xl, dtype=np.float32)
    y = np.array(yl, dtype=np.float32)
    z = np.array(zl, dtype=np.float32)
    # shift the x, y, z points so they have a zero mean 
    xm = np.mean(x)
    ym = np.mean(y)
    zm = np.mean(z)
    x = x - xm
    y = y - ym
    z = z - zm
    ones = np.ones(len(x))
    G = np.column_stack((x, y, z, ones))
    # http://stackoverflow.com/questions/10900141/fast-plane-fitting-to-many-points
    [u, s, vh] = np.linalg.svd(G, full_matrices= False)
    # get transpose of vh
    v = vh.T
    # apply an offset for the D term of the plane equation, so the plane equation is relative
    # to the orginal coordinate frame orgin.
    v[3] = v[3] - v[0]*xm - v[1]*ym - v[2]*zm
    # plane equation is the larst column of the matrix v
    return v[:,3]

def slope_plane(N):
    '''
        Determine the slope of a fit plane, which is the angle between the fit plane and the xy plane
        where N is the normal vector of the fit plane

        returns an angle in radians between 0 and pi 
    '''
    mag_N = np.linalg.norm(N, ord=2)
    # vector normal to the horizontal plane 
    B = np.array([0, 0, 1])
    return m.acos(np.dot(N,B)/mag_N)

def z_value_plane(x, y, P):
    '''
        Determine the z value of a plane, for a given x and y and the plane equation
    '''
    a, b ,c ,d = P[0], P[1], P[2], P[3]
    z = -a/c*x - b/c*y -d/c
    return z


def split_points(points):
    x, y, z = [], [], []
    for point in points:
        x.append(point.x)
        y.append(point.y)
        z.append(point.z)
    return [x, y, z]
