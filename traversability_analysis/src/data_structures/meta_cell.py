#!/usr/bin/env python

'''
    Cell data structure for storing metric results
'''

class MetaCell(object):
    def __init__(self):
        
        self.known = False
        self.slope = 0
        self.height_metric = 0
        self.roughness_metric = 0

    def set_metrics(self, slope, height, roughness):
        self.known = True
        self.slope = slope
        self.height_metric = height
        self.roughness_metric = roughness


