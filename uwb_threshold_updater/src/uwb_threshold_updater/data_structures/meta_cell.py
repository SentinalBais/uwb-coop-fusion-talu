#!/usr/bin/env python

'''
    Cell data structure for storing metric results for UWB_threshold_updater package  
'''

class MetaCell(object):
	def __init__(self):
		self.known = False
		self.slope = 0
		self.normal = []

	def set_metrics(self, slope, normal):
		self.known = True
		self.slope = slope
		self.normal = normal



