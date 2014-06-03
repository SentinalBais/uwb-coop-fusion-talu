import numpy as np
import math as m

class Point(object):
    def __init__(self, x=0.0, y=0.0, z=0.0, intensity=255, cart=True, range_m=0, az=0, elv=0):
        # http://www.mathworks.com.au/help/matlab/ref/cart2sph.html
        DEG2RAD = m.pi/180.0
        if cart == True:
            self.x = x
            self.y = y
            self.z = z
            self.intensity = intensity
            self.range = np.sqrt(x**2 + y**2 + z**2)
            self.azimuth = np.arctan2(y, x)
            r_xy = np.sqrt(x**2 + y**2)
            self.elvation = np.arctan2(z, r_xy )
        else:
            elv = elv*DEG2RAD
            az = az*DEG2RAD
            self.x = range_m*np.cos(elv)*np.cos(az)
            self.y = range_m*np.cos(elv)*np.sin(az)
            self.z = range_m*np.sin(elv)
            self.range = range_m
            self.azimuth = az
            self.elvation = elv
            self.intensity = intensity

    def __str__(self):
        line = [str(self.x), str(self.y), str(self.z)]
        print_str = ' '.join(line)
        return print_str

