#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import scipy.linalg
from dynamics import*


class control:
	
	U1 = 0
	U2 = 0
	U3 = 0
	U4 = 0
	Ux = 0
	Uy = 0
	
	U_rotors = np.zeros((4,1))
	V_rotors = np.zeros((4,1))


	def __init__(self,X,Y,Z,Yaw,quad,clog):
		np.set_printoptions(precision=3)
		self.altitude(Z,quad,clog)
		self.position(X,Y,quad,clog)
		self.attitude(Yaw,quad,clog)
	
		self.mixer(self.U1,self.U2,self.U3,self.U4)
		self.rotors(quad,clog)
		self.set_control_signals(quad)
		#print(quad.control_signal[0])
	
	def altitude(self,Z,quad,clog):
		ez = Z - quad.X[5]
		edz = clog.z.control(ez) - quad.dX[5]
		self.U1 = clog.dz.control(edz)
	
	def position(self,X,Y,quad,clog):
		ex = X - quad.X[3]
		edx = clog.x.control(ex) - quad.dX[3]
		self.Ux = clog.dx.control(edx)
		
		ey = Y - quad.X[4]
		edy = clog.y.control(ey) - quad.dX[4]
		self.Uy = clog.dy.control(edy)
	
	def attitude(self,Yaw,quad,clog):
		eroll = self.Uy - quad.X[0]
		edroll = clog.roll.control(eroll) - quad.dX[0]
		self.U2 = clog.droll.control(edroll)
		
		epitch = self.Ux - quad.X[1]
		edpitch = clog.pitch.control(epitch) - quad.dX[1]
		self.U3 = clog.dpitch.control(edpitch)
		
		eyaw = Yaw - quad.X[2]
		edyaw = clog.yaw.control(eyaw) - quad.dX[2]
		self.U4 = clog.dyaw.control(edyaw)
		
		
	def mixer(self,U1,U2,U3,U4):
	
		self.U_rotors[0] = U1 + U3 + U4
		self.U_rotors[1] = U1 - U3 + U4
		self.U_rotors[2] = U1 - U2 - U4
		self.U_rotors[3] = U1 + U2 - U4
		
		self.saturation(1000)
		
		self.U_rotors[2] = - self.U_rotors[2]
		self.U_rotors[3] = - self.U_rotors[3]
	
	def rotors(self,quad,clog):
		
		ew = self.U_rotors - quad.qd
		self.V_rotors = clog.rotors.control(ew)
		
		
	def set_control_signals(self,quad):
		quad.set_cont(self.V_rotors)
		dynamics(self.U_rotors,quad)
		
	def saturation(self, n):
		if (self.U_rotors[0])>n:
			self.U_rotors[0] = n
		if (self.U_rotors[1])>n:
			self.U_rotors[1] = n
		if (self.U_rotors[2])>n:
			self.U_rotors[2] = n
		if (self.U_rotors[3])>n:
			self.U_rotors[3] = n
		
		if (self.U_rotors[0])<0:
			self.U_rotors[0] = 0
		if (self.U_rotors[1])<0:
			self.U_rotors[1] = 0
		if (self.U_rotors[2])<0:
			self.U_rotors[2] = 0
		if (self.U_rotors[3])<0:
			self.U_rotors[3] = 0


