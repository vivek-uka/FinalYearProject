from math import pi, atan2, sqrt, cos 
import sympy as sp
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib import colors

def dist(x1,y1,x2,y2):
	return sqrt((x1-x2)**2 + (y1-y2)**2)

def angle(x1,y1,x2,y2):
	Angle = atan2((y2-y1),(x2-x1))
	return Angle

def angleNP(x1,y1,x2,y2):
	Angle = np.arctan2((y2-y1),(x2-x1))
	return Angle

def scale360(angle):
	if angle > 2*pi:
		angle = angle - 2*pi
	elif angle < -2*pi:
		angle = angle + 2*pi
	return angle

def scale180(angle):
	if angle > pi:
		angle = angle - 2*pi
	elif angle < -pi:
		angle = angle + 2*pi
	return angle

def distNP(x1,y1,x2,y2):
	return np.sqrt((x1-x2)**2 + (y1-y2)**2)

def distSP(x1,y1,x2,y2):
	return sp.sqrt((x1-x2)**2 + (y1-y2)**2)

def hueristic(i,j,xg,yg):
	return dist(i,j,xg,yg)

def plotcontour3D(X,Y,Z):
	fig= plt.figure()	
	ax = plt.axes(projection='3d')
	ax.contour3D(X, Y, Z, 50, cmap='binary')	
	#ax.plot_surface(X, Y, Z, rstride=1, cstride=1,cmap='viridis', edgecolor='none')	
	#ax.contourf3D(X,Y,Z)
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z');
	ax.set_title('surface');
	plt.show()

def plotcontour2D(X,Y,Z):
	fig, ax = plt.subplots(figsize=(6,6))
	#cp = ax.contourf(X,Y,Z)	
	cp = ax.contour(X,Y,Z)	
	ax.clabel(cp, inline=True, fontsize=10)
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_title('surface');
	plt.show()
	
def addContour(X,Y,Z1,Z2,Z3):
	for i in range(0,len(Z3)):
		for j in range(0,len(Z3)):
			Z3[i][j] =  Z2[i][j] + Z1[i][j] + Z3[i][j] 
	return X,Y,Z3		
		

def plotGrid(data,cmd):
	
	H = np.array(data)

	if cmd == "draw":
				
		plt.imshow(H,interpolation='none')	
		plt.draw()
		plt.pause(0.05)
	else:
		plt.close()
		fig2 = plt.figure()
		fig2.clf()		
		plt.imshow(H,interpolation='none')
		plt.show()
