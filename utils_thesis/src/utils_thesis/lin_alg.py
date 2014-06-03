#!/usr/bin/env python
import numpy as np 
import math as m 


def angle_between_vectors(va = 'vector1' , vb = 'vector2'):
	'''
	Finds the angle between two vectors in degrees 
	(Internal Function)

	Args:
		va: first vector numpy array [x, y, z]
		vb: second vector numpy array [x, y, z]
	Returns:
		Angle: angle between vector_a and vector_b in degrees
	'''
	RAD2DEG = 180/m.pi
	mag_va = np.linalg.norm(va) #  sum(abs(v)**2)**(1./2)
	mag_vb = np.linalg.norm(vb)
	angle = m.acos(np.dot(va,vb)/mag_va/mag_vb)*RAD2DEG 
	return angle