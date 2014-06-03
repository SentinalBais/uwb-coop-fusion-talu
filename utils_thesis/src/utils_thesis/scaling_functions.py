#!/usr/bin/env python

def inverse_parabola(alpha, theta):
	'''
		Applies Inverse Parabola Scalling 

		Returns a number between 1 and 0.5  
	'''

	return 1 - (2.0*alpha*alpha)/(theta*theta)

