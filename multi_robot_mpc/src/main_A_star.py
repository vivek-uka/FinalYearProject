#!/usr/bin/env python

from myClasses import *
from basics import *
#___________________________________________ex__Main___________________

   #0 1 2 3 4 5 6 7 8 9
#0 [_ _ _ # _ _ _ _ _ _] 
#1 [_ _ _ # _ _ _ # _ _]
#2 [_ _ # # _ _ _ # _ _]
#3 [_ _ # # _ _ G # _ _]
#4 [_ _ # # # _ _ _ _ _]
#5 [_ _ # # # # _ _ _ _]
#6 [_ _ _ _ # # _ _ _ _]
#7 [_ _ _ _ # # _ _ _ _]
#8 [_ _ _ _ _ # _ # _ _]
#9 [_ _ _ _ _ # _ _ _ _]
#10[S _ _ _ _ # _ _ _ _] Start = (10,0) Goal(3,6)
Map = [				  
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,1,1,1,1,1,1,0,0],
	[0,0,0,0,0,0,0,1,0,0],
	[0,0,0,0,0,0,0,1,0,0],
	[0,0,0,0,0,0,0,1,0,0],
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0],
	[0,0,0,0,0,0,0,0,0,0]
]


OPEN = []
OPEN_ = []
CLOSED = []
CLOSED_ = []

Point_i = [1,-1,0,0,-1,-1,1,1]
Point_j = [0,0,1,-1,-1,1,1,-1]

def smallest_F(data,minimum):
	
	index = -1

	for i in range(len(data)):
		if data[i].f < minimum:
			minimum = data[i].f 
			index = i

	return index
		
def notIn(s,data):
	
	for i in range(len(data)):
		if data[i] == s:
			return i
	return -1

def extractIndicies(s):
	i = ""
	j = ""
	space = []
	for k in range(len(s)):
		if s[k] == ' ':
			space.append(k)
	i = i + s[space[0]:space[1]]
	j = j + s[space[1]:]

	return int(i),int(j)


def  computePath(xs,ys,xg,yg,e,Map):
	
	global OPEN, CLOSED	
	global OPEN_, CLOSED_
	global Point_i, Point_j

	OPEN = []
	OPEN_ = []
	CLOSED = []
	CLOSED_ = []

	startNode = node(xs,ys,Map[xs][ys],0,xg,yg,"null",e)#i,j,val,g,xg,yg,backP,e
	startNode.backpointer = startNode.name
	OPEN.append(startNode)
	OPEN_.append(startNode.name)

	goalExpanded = 0 
	expansions = 0
	solutionExist = 0
	goalBackPointer = "null"

	temp = Map
	temp[xs][ys] = 2
	temp[xg][yg] = 3
	while(len(OPEN)!=0 and goalExpanded == 0):
		
		index = smallest_F(OPEN,1000)

		print "PUSH IN CLOSED = ",OPEN[index].name,OPEN[index].f
		i = OPEN[index].i
		j = OPEN[index].j
		g = OPEN[index].g	
		name = OPEN[index].name

		CLOSED.append(OPEN.pop(index))
		CLOSED_.append(OPEN_.pop(index))

		for k in range(len(Point_i)):
			m = Point_i[k] + i
			n = Point_j[k] + j
			
			if m < len(Map) and m >= 0 and n < len(Map[0]) and n >= 0:
				s = "s" + " " + str(m) + " " + str(n)
					
				cost = 1				
				if m!=0 and n!=0:
					cost = 1.4

				openIndex = notIn(s,OPEN_)
				closedIndex = notIn(s,CLOSED_) 
				
				if openIndex != -1:
					OPEN[openIndex].checkG(Map[m][n],g,name,e)#val,g,backP,e			
				elif closedIndex == -1:
					OPEN.append(node(m,n,Map[m][n],g+cost,xg,yg,name,e))
					OPEN_.append(s)
					expansions = expansions + 1
					
					if Map[m][n] == 0:
						temp[m][n] = 4
				if s == "s" + " " + str(xg) + " " + str(yg):
					goalExpanded = 1			
					solutionExist = 1
					goalBackPointer = name
				
				

		plotGrid(temp,"draw")
	if solutionExist:
		print "Expansions = ",expansions
		return goalBackPointer
	else:
		print "sorry no solution"
		return "null" 
		
		
							
def getSolution(xs,ys,xg,yg,goalBackPointer,Map):
	
	global CLOSED_, CLOSED

	solved = Map
	solution = []

	goal = "s"+" "+str(xg)+" "+str(yg)
	start = "s"+" "+str(xs)+" "+str(ys)

	solution.append(goal)	
	i, j = extractIndicies(goal)
	solved[i][j] = 3

	backP = goalBackPointer
	while(backP != "s"+" "+str(xs)+" "+str(ys)):
		
		solution.append(backP)
		i, j = extractIndicies(backP)
		solved[i][j] = 3	

		index = notIn(backP,CLOSED_)
		backP = CLOSED[index].backpointer
		
	solution.append(start)
	i, j = extractIndicies(start)
	solved[i][j] = 2
	
	return solved		
			
		
def A_Star(xs,ys,xg,yg,e,Map):
	goalBackPointer = computePath(xs,ys,xg,yg,e,Map)
	solution = getSolution(xs,ys,xg,yg,goalBackPointer,Map)	
	plotGrid(solution,"plot")	

		
		

if __name__ == '__main__':
           A_Star(10,0,0,9,2.5,Map)





           	

	





