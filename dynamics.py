#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import scipy.linalg


class dynamics:
	qd = np.empty([4, 1])
	Fb = np.empty([6, 1])
	X = np.empty([6, 1])
	dX = np.empty([6, 1])
	ddX = np.empty([6, 1])

	def n_int(self,Y0,dY0,dY,dt):
		return (Y0 + ((dY+dY0)/2)*dt)
	
	
	def __init__(self,voltaje,quad):
		self.X = quad.X
		self.dX = quad.dX
		self.ddX = quad.ddX
		self.rotor_dynamics(voltaje,quad)
		self.f_aero(quad)
		self.quad_dynamics(quad)
		self.set_state_vars(quad)
	
	def skew(self, v):
		return np.matrix([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
	
	def rotor_dynamics(self,voltaje,quad):
		self.qd = voltaje#quad.qd*(np.exp(-quad.ts/(quad.tau_rotor))) + voltaje*(quad.k_rotor*(1-np.exp(-quad.ts/(quad.tau_rotor))))
	
	def f_aero(self,quad):
		
		F = (quad.CL-quad.CD)*(0.5*quad.A_aspa*quad.rho_air)*(self.qd*quad.r1)**2 
		#Diferencia entre la fuerza de drag y de lift
		
		Fn = np.sum(F) #Fuerza Neta
		
		self.Fb[0] = quad.lr*(F[3]-F[2])		#Torque en roll
		self.Fb[1] = quad.lr*(F[0]-F[1])		#Torque en picth
		self.Fb[2] = quad.lr*(F[2]+F[3]-F[0]-F[1])	#Torque en yaw
		self.Fb[3] = (np.sin(quad.yaw)*np.sin(quad.roll)+np.cos(quad.yaw)*np.sin(quad.pitch)*np.cos(quad.roll))*Fn
		self.Fb[4] = (-np.cos(quad.yaw)*np.sin(quad.roll)+np.sin(quad.yaw)*np.sin(quad.pitch)*np.cos(quad.roll))*Fn
		self.Fb[5] = quad.m_robot*quad.g - np.cos(quad.pitch)*np.cos(quad.roll)*Fn
		

	def quad_dynamics(self,quad):
		sk = self.skew(self.dX[0:3])
		D = np.vstack((np.hstack((sk,quad.C)),np.hstack((quad.C,sk))))
		self.ddX = np.linalg.inv(quad.Ib)*(self.Fb - (D*quad.Ib)*self.dX)
		#Si hay 
		if self.X[5] >= 0 :
			#self.X[0:3] = np.zeros((3,1)) 
			if np.sign(self.ddX[5]) == 1:
				self.ddX[5] = 0
				self.dX[5] = 0
				self.X[5] = 0
		
	def set_state_vars(self,quad):
		self.dX = self.n_int(quad.dX,quad.ddX,self.ddX,quad.ts)
		self.X = self.n_int(quad.X,quad.dX,self.dX,quad.ts)
		quad.set_vars(self.ddX,self.dX,self.X,self.qd)
