#!/usr/bin/env python
import numpy as np 
from math import pi, atan2, degrees, sqrt

class grade_estimation:
	def __init__(self):



	def calcGrade(self, east_vel, north_vel, up_vel)

		
		long_vel = sqrt(pow(east_vel,2) + pow(north_vel,2))
		grade_estimate = atan2(up_vel,long_vel)

		return degrees(grade_estimate)
