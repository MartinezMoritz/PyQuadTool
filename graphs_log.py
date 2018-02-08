#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import pyqtgraph.opengl as gl

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.console
import numpy as np

class data_log:
	
	t = np.empty(100)
	x = np.empty(100)
	y = np.empty(100)
	z = np.empty(100)
	yaw = np.empty(100)
	pitch = np.empty(100)
	roll = np.empty(100)
	speed = np.empty(100)
	qd1 = np.empty(100)
	qd2 = np.empty(100)
	qd3 = np.empty(100)
	qd4 = np.empty(100)
	xd = np.empty(100)
	yd = np.empty(100)
	zd = np.empty(100)
	ptr = 0

	def update_data(self,quad,traj,X,Y,Z):
		
		
		self.t[ptr] = quad.time
		self.x[ptr] = quad.X[3]
		self.y[ptr] = quad.X[4]
		self.z[ptr] = quad.X[5]
		roll[ptr] = quad.X[0]
		self.pitch[ptr] = quad.X[1]
		self.yaw[ptr] = quad.X[2]
		self.speed[ptr] = np.sqrt(quad.dX[3]**2+quad.dX[4]**2+quad.dX[5]**2)
		self.qd1[ptr] = quad.self.qd[0]
		self.qd2[ptr] = quad.self.qd[1]
		self.qd3[ptr] = quad.self.qd[2]
		self.qd4[ptr] = quad.self.qd[3]
		self.xd[ptr] = X
		self.yd[ptr] = Y
		self.zd[ptr] = -traj.Z [nn]

		self.ptr += 1

		if self.ptr >= self.t.shape[0]:
			tmp = self.t
			self.t = np.empty(self.t.shape[0] * 2)
			self.t[:tmp.shape[0]] = tmp
		
			tmp = self.x
			self.x = np.empty(self.x.shape[0] * 2)
			self.x[:tmp.shape[0]] = tmp
		
			tmp = self.y
			self.y = np.empty(self.y.shape[0] * 2)
			self.y[:tmp.shape[0]] = tmp
		
			tmp = self.z
			self.z = np.empty(self.z.shape[0] * 2)
			self.z[:tmp.shape[0]] = tmp
		
			tmp = self.yaw
			self.yaw = np.empty(self.yaw.shape[0] * 2)
			self.yaw[:tmp.shape[0]] = tmp
		
			tmp = self.pitch
			self.pitch = np.empty(self.pitch.shape[0] * 2)
			self.pitch[:tmp.shape[0]] = tmp
		
			tmp = roll
			roll = np.empty(roll.shape[0] * 2)
			roll[:tmp.shape[0]] = tmp

			tmp = self.speed
			self.speed = np.empty(self.speed.shape[0] * 2)
			self.speed[:tmp.shape[0]] = tmp

			tmp = self.qd1
			self.qd1 = np.empty(self.qd1.shape[0] * 2)
			self.qd1[:tmp.shape[0]] = tmp

			tmp = self.qd2
			self.qd2 = np.empty(self.qd2.shape[0] * 2)
			self.qd2[:tmp.shape[0]] = tmp

			tmp = self.qd3
			self.qd3 = np.empty(self.qd3.shape[0] * 2)
			self.qd3[:tmp.shape[0]] = tmp

			tmp = self.qd4
			self.qd4 = np.empty(self.qd4.shape[0] * 2)
			self.qd4[:tmp.shape[0]] = tmp

			tmp = self.xd
			self.xd = np.empty(self.xd.shape[0] * 2)
			self.xd[:tmp.shape[0]] = tmp

			tmp = self.yd
			self.yd = np.empty(self.yd.shape[0] * 2)
			self.yd[:tmp.shape[0]] = tmp

			tmp = self.zd
			self.zd = np.empty(self.zd.shape[0] * 2)
			self.zd[:tmp.shape[0]] = tmp

	def quad_traj(self):
		return np.vstack([x[:ptr], y[:ptr], -z[:ptr]]).transpose()

	def set_curves(self,curve):
		
		curve.t.setData(t[:self.ptr],x[:self.ptr])
		curve.y.setData(t[:self.ptr],y[:self.ptr])
		curve.z.setData(t[:self.ptr],z[:self.ptr])
		curve.yaw.setData(t[:self.ptr],yaw[:self.ptr])
		curve.pitch.setData(t[:self.ptr],pitch[:self.ptr])
		curve.roll.setData(t[:self.ptr],roll[:self.ptr])
		curve.speed.setData(t[:self.ptr],speed[:self.ptr])
		curve.qd1.setData(t[:self.ptr],qd1[:self.ptr])
		curve.qd2.setData(t[:self.ptr],qd2[:self.ptr])
		curve.qd3.setData(t[:self.ptr],qd3[:self.ptr])
		curve.qd4.setData(t[:self.ptr],qd4[:self.ptr])
		curve.xd.setData(t[:self.ptr],xd[:self.ptr])
		curve.yd.setData(t[:self.ptr],yd[:self.ptr])
		curve.zd.setData(t[:self.ptr],zd[:self.ptr])

