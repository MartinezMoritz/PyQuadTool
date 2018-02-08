#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import scipy.linalg
from dynamics import*

class PID:
	
	e2 = 0
	e1 = 0
	e0 = 0
	u2 = 0
	u1 = 0
	u0 = 0

	def __init__(self,kp,ki,kd,N,Ts):
		
		div = (2*N*Ts + 4)
		self.a = (4*kp + 4*N*kd + 2*Ts*ki + 2*N*Ts*kp + N*(Ts**2)*ki)/div
		self.b = (2*N*ki*Ts**2 - 8*kp - 8*N*kd)/div
		self.c =  (4*kp + 4*N*kd - 2*Ts*ki - 2*N*Ts*kp + (N*Ts**2)*ki)/div
		self.d = -8/div
		self.f = (-2*N*Ts + 4)/div

		
	
	def control(self,error):
		
		self.e2 = self.e1
		self.e1 = self.e0
		self.u2 = self.u1
		self.u1 = self.u0
		self.e0 = error
		
		self.u0 = - self.d*self.u1 - self.f*self.u2 + self.c*self.e2 + self.b*self.e1 + self.a*self.e0
		return self.u0

class PI:
	
	e1 = np.zeros((4,1))
	e0 = np.zeros((4,1))
	u1 = np.zeros((4,1))
	u0 = np.zeros((4,1))

	def __init__(self,Kp,Ki,Ts):
		self.Kp = Kp
		self.Ki = Ki
		self.Ts = Ts

	def control(self,error):
		
		self.e1 = self.e0
		self.u1 = self.u0
		self.e0 = error
		
		self.u0 = self.u0 + self.Kp*self.e0 - self.Kp*self.e1 + self.Ki*self.Ts*self.e1
		
		self.saturation()
		
		return self.u0

	def saturation(self):
		if abs(self.u0[0])>11.1:
			self.u0[0] = (self.u0[0]/abs(self.u0[0]))*11.1
		if abs(self.u0[1])>11.1:
			self.u0[1] = (self.u0[1]/abs(self.u0[1]))*11.1
		if abs(self.u0[2])>11.1:
			self.u0[2] = (self.u0[2]/abs(self.u0[2]))*11.1
		if abs(self.u0[3])>11.1:
			self.u0[3] = (self.u0[3]/abs(self.u0[3]))*11.1


# CONTROLES
class Control_log:

	def __init__(self,Ts):
		kp = 6.98646505660333
		ki = 2.80588845504253
		kd = -0.0719987939256043
		N = 97.0358623482275
		self.droll = PID(kp,ki,kd,N,Ts)

		kp = 21.4591778402582
		ki = 4.06510287297299
		kd = -0.00867413435250052
		N = 2473.92730711764
		self.roll = PID(kp,ki,kd,N,Ts)

		kp = 6.98646505660333
		ki = 2.80588845504253
		kd = -0.0719987939256043
		N = 97.0358623482275
		self.dpitch = PID(kp,ki,kd,N,Ts)

		kp = 21.4591778402582
		ki = 4.06510287297299
		kd = -0.00867413435250052
		N = 2473.92730711764
		self.pitch = PID(kp,ki,kd,N,Ts)

		kp = -4.60500464904922
		ki = -2.438052110357
		kd = 0.0359996106093005
		N = 127.918179422183
		self.dyaw = PID(kp,ki,kd,N,Ts)

		kp = 21.4591778402582
		ki = 4.06510287297299
		kd = -0.00867413435250052
		N = 2473.92730711764
		self.yaw = PID(kp,ki,kd,N,Ts)

		kp = -1569.81101442029
		ki = -1186.83791749743
		kd = 10.6144308808649
		N = 147.894035209204
		self.dz = PID(kp,ki,kd,N,Ts)

		kp = 5.25607182220264
		ki = 0.220905787961439
		kd = -0.517602217541856
		N = 10.1546547601056
		self.z = PID(kp,ki,kd,N,Ts)

		kp = -1.00482175199573
		ki = -0.14161966849369
		kd = -0.0151437459198144
		N = 1844.68318645264
		self.dy = PID(kp,ki,kd,N,Ts)

		kp = 7.02899767359397
		ki = 0.436456212719827
		kd = -0.00866797492940213
		N = 810.915782618536;
		self.y = PID(kp,ki,kd,N,Ts)

		kp = 1.01946545263466
		ki = 0.14606087647463
		kd = 0.0159539762089208
		N = 1875.4272416284;
		self.dx = PID(kp,ki,kd,N,Ts)

		kp = 7.19765327936102
		ki = 0.457502412191704
		kd = -0.0086708162630603
		N = 830.101003295929
		self.x = PID(kp,ki,kd,N,Ts)
		
		kp = 1.4935
		ki = 10.218
		self.rotors = PI(kp,ki,Ts)


