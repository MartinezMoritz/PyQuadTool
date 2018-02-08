#!/usr/bin/env python
# -*- coding: utf-8 -*-



import numpy as np

class Model:
	"""Carga un modelo en 3D a partir de un archivo .OBJ y genera una matriz con la información del Mesh.
	
	A partir del archivo, importa la información de caras y vértices:
	
	v 'x'  'y'  'z'
	f 'v1' 'v2' 'v3'
	
	Este código se ha basado en el trabajo del usuario 'curvedinfinity' de http://ubuntuforums.org/
	en un ejemplo de importación de objetos 3D en formato .OBJ: http://ubuntuforums.org/showthread.php?t=1037392
	
	Parámetros:
	filepath -- ubicación y nombre del archivo .OBJ
	
	Variables:
	
	triangles -- 
	
	"""
	triangles = []
	vertices= []
	tam = 0
	def __init__(self,filepath):
		self.loadObj(filepath)
		self.setVerts()
		self.returnMesh()
		
	
	def loadObj(self,filepath):
		'Carga el archivo .OBJ y organiza los vertices indexados por caras'
		modelFile = open(filepath,"r")
		triangles = []
		vertices = []
		for line in modelFile.readlines():
			line = line.strip()
			if len(line)==0 or line.startswith("#"):
				continue
			data = line.split(" ")
			if data[0]=="v":
				vertices.append((float(data[1]),float(data[2]),float(data[3])))
			if data[0]=="f":
				vertex1 = vertices[int(data[1].split("/")[0])-1]
				vertex2 = vertices[int(data[2].split("/")[0])-1]
				vertex3 = vertices[int(data[3].split("/")[0])-1]
				triangles.append((vertex1,vertex2,vertex3))
				self.tam+=1
		self.triangles = triangles
		self.verts=np.empty((self.tam, 3, 3), dtype=np.float32)
		'Genera una matriz de (tam, 3, 3), donde tam es el número de caras. Cada cara está compuesta de 3 vértices'
#	
	def setVerts(self):
		'Llena la matriz de vértices para cada cara triangular'
		i = 0
		for triangle in self.triangles:
			self.verts[i,0,:]=(triangle[0][0],triangle[0][1],triangle[0][2])
			self.verts[i,1,:]=(triangle[1][0],triangle[1][1],triangle[1][2])
			self.verts[i,2,:]=(triangle[2][0],triangle[2][1],triangle[2][2])
			i+=1
	
	def returnMesh(self):
		'Retorna la matriz de vértices para que pueda ser usada como MeshData'
		return self.verts

