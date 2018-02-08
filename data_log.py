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

	def update_data(self,quad,X,Y,Z):
		
		
		self.t[self.ptr] = quad.time
		self.x[self.ptr] = quad.X[3]
		self.y[self.ptr] = quad.X[4]
		self.z[self.ptr] = quad.X[5]
		self.roll[self.ptr] = quad.X[0]
		self.pitch[self.ptr] = quad.X[1]
		self.yaw[self.ptr] = quad.X[2]
		self.speed[self.ptr] = np.sqrt(quad.dX[3]**2+quad.dX[4]**2+quad.dX[5]**2)
		self.qd1[self.ptr] = quad.qd[0]
		self.qd2[self.ptr] = quad.qd[1]
		self.qd3[self.ptr] = quad.qd[2]
		self.qd4[self.ptr] = quad.qd[3]
		self.xd[self.ptr] = X
		self.yd[self.ptr] = Y
		self.zd[self.ptr] = Z

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
		
			tmp = self.roll
			self.roll = np.empty(self.roll.shape[0] * 2)
			self.roll[:tmp.shape[0]] = tmp

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
		return np.vstack([self.x[:self.ptr], self.y[:self.ptr], -self.z[:self.ptr]]).transpose()

	def set_curves(self,curvex,curvey,curvez,curvexd,curveyd,curvezd,curveyaw,curveroll,curvepitch,curveqd1,curveqd2,curveqd3,curveqd4,curvespeed):
		
		curvex.setData(self.t[:self.ptr],self.x[:self.ptr])
		curvey.setData(self.t[:self.ptr],self.y[:self.ptr])
		curvez.setData(self.t[:self.ptr],self.z[:self.ptr])
		curveyaw.setData(self.t[:self.ptr],self.yaw[:self.ptr])
		curvepitch.setData(self.t[:self.ptr],self.pitch[:self.ptr])
		curveroll.setData(self.t[:self.ptr],self.roll[:self.ptr])
		curvespeed.setData(self.t[:self.ptr],self.speed[:self.ptr])
		curveqd1.setData(self.t[:self.ptr],self.qd1[:self.ptr])
		curveqd2.setData(self.t[:self.ptr],self.qd2[:self.ptr])
		curveqd3.setData(self.t[:self.ptr],self.qd3[:self.ptr])
		curveqd4.setData(self.t[:self.ptr],self.qd4[:self.ptr])
		curvexd.setData(self.t[:self.ptr],self.xd[:self.ptr])
		curveyd.setData(self.t[:self.ptr],self.yd[:self.ptr])
		curvezd.setData(self.t[:self.ptr],self.zd[:self.ptr])

