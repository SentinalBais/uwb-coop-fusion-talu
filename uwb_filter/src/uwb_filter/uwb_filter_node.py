#!/usr/bin/env python 
import rospy
import math as m
import time
from uwb_radar_msgs.msg import UWBScan
from utils_thesis.common import *
import numpy as np
import pyqtgraph as pg

'''
uwb_data fusion node 

'''
# GLOBAL CONSTANTS
DEG2RAD = m.pi/180.0

class CallBackClass(object):
    def __init__(self, rmin=5.1, rmax=16.5):

        self.radar_spectrum = []
        self.radar_indexes = []
        # determine the indexes we want to visualise 
        self.index_start = int(rmin/0.3) - 1
        self.index_end = int(rmax/0.3) + 1


    def uwb_callback(self, data):
        self.radar_spectrum = data.intensity
        if len(self.radar_indexes) == 0:
            self.radar_indexes = np.arange(0, len(data.intensity), 1)
              
## end callback class 
def main():
    rospy.init_node('uwb_filter')
    # set any required ROS parameters 
    rospy.set_param('use_sim_time', True)
    # -------- init tf listener and all Publishers------------------------------------------------------

    # --------------------------------------------------------------------------------------------------
    # instantiate callback class 
    callback_manager = CallBackClass()
    #----------init subscribers -------------------------------------------------------------------------
    rospy.Subscriber("radar_scan", UWBScan, callback_manager.uwb_callback)
    
    PW = pg.plot()
    while not rospy.is_shutdown(): 
        # visualise the current radar scan 
        radar_spectrum = callback_manager.radar_spectrum
        if len(callback_manager.radar_indexes) > 0:
            radar_spectum_rel = radar_spectrum[callback_manager.index_start:callback_manager.index_end]
            x = np.arange(callback_manager.index_start, callback_manager.index_end, 1)*0.3
            PW.plot(radar_spectum_rel, x=x, clear=True)
            # visualise whole spectrum 
            # x = callback_manager.radar_indexes*0.3 
            # PW.plot(radar_spectrum, x=x, clear=True)
            pg.QtGui.QApplication.processEvents()
        time.sleep(0.1)
    # node shutdown procedure 

if __name__ == '__main__':
    main()


