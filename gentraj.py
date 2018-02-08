from __future__ import print_function
import numpy as np
from traj_planner import *

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

class gentraj():
	ts = 0.01
	def __init__(self,num):
		
		if num==1:
			P0 = np.array([0, 0, 0])
			PT = np.array([0, 0, 1])
			pt = 0.3
			vk = 1
		
			Type = 1
			curve = np.vstack((P0,PT))
		
			arg1 = curve
			arg2 = vk
			arg3 = pt
			arg4 = self.ts

			traj1 = traj_planner(Type, arg1, arg2, arg3, arg4)

			P0 = np.array([0, 0, 1])
			PT = np.array([4, 0, 1])
			curve = np.vstack((P0,PT))
			arg2 = 2
			arg1 = curve
			traj2 = traj_planner(Type, arg1, arg2, arg3, arg4)

			traj3 = traj_planner(3, traj1, traj2, 0, 0)

			P0 = np.array([4, 0, 1])
			PT = np.array([4, 0, 0])
			curve = np.vstack((P0,PT))
			arg2 = 0.5
			arg1 = curve
			traj4 = traj_planner(Type, arg1, arg2, arg3, arg4)

			self.traj = traj_planner(3, traj3, traj4, 0, 0)

		if num==2:
			P0 = np.array([0, 0, 0])
			PT = np.array([3, 0, 3])
			pt = 0.3
			vk = 1
		
			Type = 1
			curve = np.vstack((P0,PT))
		
			arg1 = curve
			arg2 = vk
			arg3 = pt
			arg4 = self.ts

			traj1 = traj_planner(Type, arg1, arg2, arg3, arg4)

			P0 = np.array([3, 0, 3])
			PT = np.array([8, 2, 0])
			curve = np.vstack((P0,PT))
			arg2 = 0.6
			arg1 = curve
			traj2 = traj_planner(Type, arg1, arg2, arg3, arg4)

			self.traj = traj_planner(3, traj1, traj2, 0, 0)


		if num==3:
			P0 = np.array([0,0,0, 0])
			P1 = np.array([0,0,2, 5])
			P2 = np.array([1,3,2.5, 7])
			P3 = np.array([-4,4,3, 10])
			P4 = np.array([-6,0,3.5, 13])
			P5 = np.array([-6,-4,3.5, 15])
			P6 = np.array([-5,0,2, 18])
			P7 = np.array([-2,0,1, 24])
			P8 = np.array([0,0,0, 30])

			curve = np.vstack((P0,P1,P2,P3,P4,P5,P6,P7,P8))

			Type=2
			arg1 = curve
			arg2 = np.array([0,0,0]) #velocidad inicial
			arg3 = np.array([0,0,0]) #Velocidad final
			arg4 = self.ts #Tiempo de muestreo

			self.traj = traj_planner(Type, arg1, arg2, arg3, arg4)







