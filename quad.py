#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import numpy as np

class quad:
	#VARIABLES DE ESTADO INICIALES
	roll = 0
	pitch = 0
	yaw = 0
	X = np.array([roll,pitch,yaw,0,0,0]).reshape(-1,1)
	dX = np.array([0,0,0,0,0,0]).reshape(-1,1)
	ddX = np.array([0,0,0,0,0,0]).reshape(-1,1)
	ts = 0.001
	time = 0
	qd = np.zeros((4,1)) #Velocidades angulares de los motores
	
	#Voltaje en los rotores
	V_rotors = np.zeros((4,1))
	
	#OPERADOR DE INERCIA
	Ib = np.zeros((6,6))

	# PARÁMETROS ARDRONE 2.0
	lr = 0.1785 # Distancia del centro de masa al centro del rotor
	r1 = 99e-3 # Radio de las aspas
	h1 = 1.1e-2 # Grosor de las aspas
	r2 = 1.6e-2 # Radio del rotor
	h2 = 0.1e-2 # Altura del rotor
	r3 = 0.050 # Radio del drone
	m_rotor = 3.55e-3 # Masa puntual del rotor
	m_robot = 0.429 # Masa del AR Drone 2
	m_cm = m_robot-4*m_rotor
	A_robot = 0.09426 # Área del robot
	A_aspa = r1*h1 #2*np.pi*r1*(r1+h1) # Área del aspa
	
	rotor_red = 0.1178571429 #reduccion del motor a las aspas
	k_rotor = 390.5763835135*rotor_red #Ganancia del motor (máximo 11,1 V)
	tau_rotor = 0.178
	
	# Parámetros Ambiente
	rho_air = 1.29 # Densidad del aire
	CL = 1.74
	CD = 0.022 #0.0549
	g = 9.81

	# Matrices Auxiliares
	U = np.identity(3)
	C = np.zeros((3,3))
	H=np.array([0,0,1,0,0,0]).reshape(-1,1)
	


	def skew(self, v):
		return np.matrix([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

	def __init__(self):
		self.calc_inertia()

	def calc_inertia(self):
		# CÁLCULO LA DINÁMICA INVERSA DEL QUADROTOR
		s_o1cm = np.array([self.lr*np.cos(np.pi/4),self.lr*np.sin(np.pi/4),0])
		s_o2cm = np.array([-self.lr*np.cos(np.pi/4),-self.lr*np.sin(np.pi/4),0])
		s_o3cm = np.array([self.lr*np.cos(np.pi/4),-self.lr*np.sin(np.pi/4),0])
		s_o4cm = np.array([-self.lr*np.cos(np.pi/4),self.lr*np.sin(np.pi/4),0])
	
		sS_o1cm=self.skew(s_o1cm)
		sS_o2cm=self.skew(s_o2cm)
		sS_o3cm=self.skew(s_o3cm)
		sS_o4cm=self.skew(s_o4cm)
	
		S_o1cm = np.vstack((np.hstack((self.U,sS_o1cm)),np.hstack((self.C,self.U))))
		S_o2cm = np.vstack((np.hstack((self.U,sS_o2cm)),np.hstack((self.C,self.U))))
		S_o3cm = np.vstack((np.hstack((self.U,sS_o3cm)),np.hstack((self.C,self.U))))
		S_o4cm = np.vstack((np.hstack((self.U,sS_o4cm)),np.hstack((self.C,self.U))))

		# OPERADOR DE INERCIA

		#Aspas
		I_axx =(np.pi*self.h1*self.r1**4)/4+(np.pi*self.h1**3*self.r1**2)/3
		I_ayy = I_axx
		I_azz = (np.pi*self.h1*self.r1**4)/2
		J_aspas = self.rho_air*np.matrix([[I_axx,0,0],[0,I_ayy,0],[0,0,I_azz]])
		

		#Rotor
		I_rxx = (self.r2**2)/4 + (self.h2**2)/3
		I_ryy = I_rxx
		I_rzz = (self.r2**2)/2
		J_rotor = self.m_rotor*np.matrix([[I_rxx,0,0],[0,I_ryy,0],[0,0,I_rzz]])

		#Extremidad
		J_oi = J_aspas + J_rotor

		#Cuerpo
		I_cxx = 0.4*self.r3**2
		I_cyy = I_cxx
		I_czz = I_cyy
		J_cm = self.m_cm*np.matrix([[I_cxx,0,0],[0,I_cyy,0],[0,0,I_czz]])



		#Operador de inercia
		self.Ib = np.vstack((np.hstack((4*sS_o1cm*J_oi*sS_o1cm.T + J_cm,self.C)),np.hstack((self.C,(self.m_rotor + self.m_cm)*self.U))))
		np.set_printoptions(precision=3)


	def set_vars(self,ddXi,dXi,Xi,qdi):
		self.X = Xi
		self.dX = dXi
		self.ddX = ddXi
		self.qd = qdi
		self.time = self.time+self.ts
		
		self.roll = self.X[0]
		self.pitch = self.X[1]
		self.yaw = self.X[2]

	def print_state(self):
		np.set_printoptions(precision=3)
		print("\nX = \n",self.X,"\nTiempo: ", self.time)

	def set_cont(self,v):
		self.V_rotors = v


