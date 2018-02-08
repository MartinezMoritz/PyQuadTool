from __future__ import print_function
import numpy as np
import scipy as Sci
import scipy.linalg

class traj_planner:
	X = []
	Y = []
	Z = []
	Time = []
	dX = []
	dY = []
	dZ = []
	ddX = []
	ddY = []
	ddZ = []
	
	
	
	def __init__(self,Type, arg1, arg2, arg3, arg4):
		if Type == 1:
			P0=arg1[0,:][np.newaxis]
			P0 = P0.T
			PT=arg1[1,:][np.newaxis]
			PT = PT.T
			vk=arg2
			pt=arg3
			ts=arg4
			# Validar el procentaje de aceleracion
			if pt<0:
				pt=-pt
				print ("Warning: El procentaje no debe ser una cantidad menor a cero, se asumira positiva")
			else:
				if pt>1:
					pt=1
					print ("Warning: El procentaje es mas alto que el 50%, se asumira al 50%")

			#Puntos intermedios
			T = np.linalg.norm(PT-P0)/(vk*(1-pt))
			T = np.float64(T)
			Tau=T*pt
			a=vk/Tau
			
			Ptau=0.5*a*Tau**2*((PT-P0)/np.linalg.norm(PT-P0))+P0
			
			PT_tau=((vk*(T-2*Tau))/np.linalg.norm(PT-P0))*(PT-P0)+Ptau
			
			#Puntos de muestreo y vectores
			n = round(T/ts)
			time = np.zeros((n+1))
			P = np.zeros((3,n+1))
			V = np.zeros((3,n+1))
			A = np.zeros((3,n+1))
			
			#Tramo acelerado:
			tau=2*np.linalg.norm(Ptau-P0)/vk
			tau = np.float64(tau)
			k=round(pt*n)
			
			t=np.arange(0,tau+tau/k,tau/k)
			#print(tau, k)


			time[0:k]=t[0:k]
			P[:,0:k+1]=(Ptau-P0)*(1/np.linalg.norm(Ptau-P0))*(0.5*a*t**2)+P0*np.ones((1,k+1))
			V[:,0:k+1]=(Ptau-P0)*(1/np.linalg.norm(Ptau-P0))*(a*t)
			A[:,0:k+1]=(Ptau-P0)*(1/np.linalg.norm(Ptau-P0))*(a*np.ones((t.size)))
			#Tramo velocidad constante:
			tau=np.linalg.norm(PT_tau-Ptau)/vk
			kc=round(n-2*k)
			
			t=np.arange(0,tau+tau/kc,tau/kc)
			
			time[k:kc+k+1]=t+Tau
			
			P[:,k:kc+k+1]=(PT_tau-Ptau)*(vk*t/np.linalg.norm(PT_tau-Ptau))+Ptau*np.ones((1,kc+1))
			V[:,k:kc+k+1]=(PT_tau-Ptau)*(vk*np.ones((t.size))/np.linalg.norm(PT_tau-Ptau))
			A[:,k:kc+k+1]=(PT_tau-Ptau)*(vk*np.zeros((t.size))/np.linalg.norm(PT_tau-Ptau))
			
			#Tramo desacelerado:
			a=-a
			
			tau=2*np.linalg.norm(PT-PT_tau)/vk
			
			k=round(pt*n)
			
			t=np.arange(tau/k,tau+tau/k,tau/k)
			
			#print(t-tau)
			time[k+kc+1:2*k+kc+1]=t+(T-Tau)
			
			P[:,k+kc+1:2*k+kc+1]=(PT-PT_tau)*(1/np.linalg.norm(PT-PT_tau))*(0.5*a*t**2+vk*t)+PT_tau*np.ones((1,k))
			V[:,k+kc+1:2*k+kc+1]=(PT-PT_tau)*(1/np.linalg.norm(PT-PT_tau))*(a*t+vk)
			A[:,k+kc+1:2*k+kc+1]=(PT-PT_tau)*(1/np.linalg.norm(PT-PT_tau))*(a*np.ones((t.size)))
			
			
			#Reconstruccion de trayectoria: 
			self.Time = (time).T
			self.X = (P[0,:]).T
			self.Y = (P[1,:]).T
			self.Z = (P[2,:]).T

			self.dX = (V[0,:]).T
			self.dY = (V[1,:]).T
			self.dZ = (V[2,:]).T

			self.ddX = (A[0,:]).T
			self.ddY = (A[1,:]).T
			self.ddZ = (A[2,:]).T
		
		if Type ==2:
			#SPLINES CUBICOS
			d = arg1[:,0:3]
			t = arg1[:,3]
			t = t.T
			V0 = arg2
			Vf = arg3
			ts = np.float32(arg4)
			n = t.size
			
			
			
			A_aux=np.zeros((n+1,n+1))
			h_aux=np.zeros((n+1,1))
			
			for i in range(n-1):
				h_aux[i+1]=t[i+1]-t[i]
			
			for i in range (n):
				A_aux[i,i]=2*(h_aux[i]+h_aux[i+1])
				A_aux[i,i+1]=h_aux[i+1]
				A_aux[i+1,i]=h_aux[i+1]
			
			F = np.zeros((n,3))
			A = np.zeros((n,n))
			h = np.zeros((n-1,1))
			
			h = h_aux[1:n,0]
			A = A_aux[0:n,0:n]
			
			
			
			F_aux = np.zeros((n+1,3))
			
			
			F_aux[0,:] = 3*V0
			F_aux[n,:] = 3*Vf
			
			for i in range(n-1):
				F_aux[i+1,:]=(3/h[i])*(d[i+1,:]-d[i,:])
			
			
			for i in range(n):
				#print(F[i,:])
				F[i,:]=F_aux[i+1,:]-F_aux[i,:]
				
			
			
			b=np.dot(np.linalg.inv(A),F)
			
			
			a=np.zeros((n-1,3))
			c=np.zeros((n-1,3))
			
			for i in range(n-1):
				a[i,:]=(b[i+1,:]-b[i,:])/(3*h[i])
				c[i,:]= (1/h[i])*(d[i+1,:]-d[i,:])-(h[i]/3)*(2*b[i,:]+b[i+1,:])
			

			ttam=1
			for tram in range(n-1):
				dt=ts
				dt = np.float32(dt)
				for i in np.arange(dt, h[tram]+dt, dt):
					ttam = ttam + 1
			
			X = np.zeros((ttam))
			Y = np.zeros((ttam))
			Z = np.zeros((ttam))
			Vx = np.zeros((ttam))
			Vy = np.zeros((ttam))
			Vz = np.zeros((ttam))
			Ax = np.zeros((ttam))
			Ay = np.zeros((ttam))
			Az = np.zeros((ttam))
			tiempo = np.zeros((ttam))
			
			b=b[0:n-1,:]
			d=d[0:n-1,:]
			
			
			
			X[0]=d[0,0]
			Y[0]=d[0,1]
			Z[0]=d[0,2]
			
			Vx[0]=V0[0]
			Vy[0]=V0[1]
			Vz[0]=V0[2]
			
			tiempo[0]=t[0]
			
			j=1
			
			
			for tram in range(n-1):
				ap=a[tram,:]
				bp=b[tram,:]
				cp=c[tram,:]
				dp=d[tram,:]
				
				
				
				dt=ts
				dt = np.float32(dt)
#				P=dp+cp*t+bp*t^2+ap*t^3;
#				V=cp+2*bp*t+3*ap*t^2;
#				A=2*bp+6*ap*t;
				
				
				for tm in np.arange(dt, h[tram]+dt, dt):
					P=dp+cp*tm+bp*tm**2+ap*tm**3
					V=cp+2*bp*tm+3*ap*tm**2
					A=2*bp+6*ap*tm
					
					X[j]=P[0]
					Y[j]=P[1]
					Z[j]=P[2]
					
					Vx[j]=V[0]
					Vy[j]=V[1]
					Vz[j]=V[2]
					
					Ax[j]=A[0]
					Ay[j]=A[1]
					Az[j]=A[2]
					
					tiempo[j]=tiempo[j-1]+dt
					j=j+1;
				
			
			Ax[0]=(Vx[1]-Vx[0])/h[0]
			Ay[0]=(Vy[1]-Vy[0])/h[0]
			Az[0]=(Vz[1]-Vz[0])/h[0]
	

			#Reconstruccion de trayectoria: 
			self.Time = tiempo
			self.X = X
			self.Y = Y
			self.Z = Z

			self.dX = Vx
			self.dY = Vy
			self.dZ = Vz

			self.ddX = Ax
			self.ddY = Ay
			self.ddZ = Az
		
		if Type == 3:
			traj1 = arg1
			traj2 = arg2
			self.X = np.hstack((traj1.X,traj2.X))
			self.Y = np.hstack((traj1.Y,traj2.Y))
			self.Z = np.hstack((traj1.Z,traj2.Z))
			self.dX = np.hstack((traj1.dX,traj2.dX))
			self.dY = np.hstack((traj1.dY,traj2.dY))
			self.dZ = np.hstack((traj1.dZ,traj2.dZ))
			self.ddX = np.hstack((traj1.ddX,traj2.ddX))
			self.ddY = np.hstack((traj1.ddY,traj2.ddY))
			self.ddZ = np.hstack((traj1.ddZ,traj2.ddZ))
			self.Time = np.hstack((traj1.Time,traj2.Time+np.amax(traj1.Time)))

