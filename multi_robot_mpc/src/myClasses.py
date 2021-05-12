from basics import dist
freq = 100
class PID:
	kp = 0
	kd = 0
	ki = 0
	required = 0 
	error = 0 
	prevError = 0
	derivative = 0
	integral = 0
	output = 0
	minControl = 0
	maxControl = 0

	def __init__(self,kp,kd,ki,required,minControl,maxControl):
		self.kp = kp
		self.kd = kd
		self.ki = ki
		self.required = required
		self.maxControl = maxControl
		self.minControl = minControl

	def pidControl(self,actual):
		 
		self.error = self.required - actual
		self.derivative = (self.error - self.prevError)*freq
		self.integral = self.integral + self.error
		self.prevError = self.error
		self.output = self.kp*self.error + self.kd*self.derivative + self.ki*self.integral
		
		if self.output > self.maxControl:
			self.output = self.maxControl
		elif self.output < self.minControl:
			self.output = self.minControl
		return self.output

class node:
	f = float('inf')
	g = float('inf')
	h = float('inf')
	v = float('inf')
	i = 0
	j = 0
	e = 1
	expanded = 1
	name = "s"
	backpointer = ""
	
	def __init__(self,i,j,val,g,xg,yg,backP,e):
		self.name = self.name + " " + str(i) +" "+ str(j)		
		if val == 1:
			if self.g > g + 100000: #obstacle
				self.g = g + 100000
		else:
			if self.g > g:	
				self.g = g	#free space
	
		self.e = e
		self.h = dist(i,j,xg,yg)
		self.f = self.g + self.e*self.h
	 	self.i = i
		self.j = j
		self.backpointer = backP

	def checkG(self,val,g,backP,e):
		self.e = e
		if val == 1:
			if self.g > g + 100000: #obstacle
				self.g = g + 100000
				self.backpointer = backP
		else:
			if self.g > g:	
				self.g = g	#free space
				self.backpointer = backP

		self.f = self.g + self.e*self.h

	def updateF(self,e):
		self.e = e
		self.f = self.g + self.e*self.h

class circle:
	h = 0
	k = 0
	r = 0

	def __init__(self,H,K,R):
		self.h = H
		self.k = K
		self.r = R

		


