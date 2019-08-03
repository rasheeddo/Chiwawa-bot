# Test servo with Maestro 

import maestro
import time
import numpy as np
import math as m
import matplotlib.pyplot as plt

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def ServoCom1(deg1):
	return int(map(deg1,0,180,8512,3648))
def ServoCom2(deg2):
	return int(map(deg2,0,180,3648,8512))

def INV(px,py):

	## Parameters
	L1 = 74
	L2 = 180
	RA = L1
	QB = L1
	AP = L2
	BP = L2
	RQ = 40

	if (py > -120):
		print("INV: input Y is positive!...")
		return [None]*2

	else:
		# Input px, py are in range
		RP = m.sqrt((px + RQ/2)**2 + py**2)
		QP = m.sqrt((px - RQ/2)**2 + py**2)

		gam1 = m.atan(( px + RQ/2 )/abs(py))
		gam2 = m.atan(( px - RQ/2 )/abs(py))

		alp1 = m.acos((AP**2 - RP**2 - RA**2)/(-2*RP*RA))
		alp2 = m.acos((BP**2 - QP**2 - QB**2)/(-2*QP*QB))

		the1 = ((alp1-gam1)*180/m.pi)
		the2 = ((alp2+gam2)*180/m.pi)

		return the1,the2

def CreateLinePath(startPoint,stopPoint,des_vel):

	global gap_time, plotTime

	Px = list()
	Py = list()
	The1 = list()
	The2 = list()
	plotTime = list()

	PX_start = startPoint[0]
	PY_start = startPoint[1]

	PX_goal = stopPoint[0]
	PY_goal = stopPoint[1]

	distance = m.sqrt( (PX_goal - PX_start)**2 + (PY_goal - PY_start)**2 )   # Total distance
	vector = [(PX_goal-PX_start), (PY_goal-PY_start)]  # a vector of path
	unit_vec = [vector[0]/distance, vector[1]/distance] # Specify the direction
 
	finish_time = distance/des_vel  		# Total time [sec] from start to goal points
	points = round(distance)
	gap = distance/points
	gap_time = finish_time/points

	print("Distance", distance)
	print("Desired Speed", des_vel)
	print("Unit vector", unit_vec)
	print("Finish time", finish_time)
	print("Points", points)
	print("Gap distance", gap)
	print("Gap time", gap_time)

	for i in range(0,int(points)):
		if i == 0:
			Px.append(PX_start + unit_vec[0]*gap)
			Py.append(PY_start + unit_vec[1]*gap)
		else:
			Px.append(Px[i-1] + unit_vec[0]*gap)
			Py.append(Py[i-1] + unit_vec[1]*gap)

		the1, the2 = INV(Px[i], Py[i])
		The1.append(the1)
		The2.append(the2)
		plotTime.append(gap_time*i)

	return The1, The2

def DrawLine(Ang1,Ang2):
	samplingTime = gap_time
	T = time.time()
	startTime = time.time()
	for j in range(0,len(Ang1)):

		T=T+samplingTime
		#print("the1",the1[j])
		#print("the2",the2[j])
		#print("------------")
		servo.setTarget(0,ServoCom1(Ang1[j]))
		servo.setTarget(1,ServoCom2(Ang2[j]))

		time.sleep(max(0,T-time.time()))     #max is needed in Windows due to

	while(servo.isMoving(0) or servo.isMoving(1)):
		pass

	totalTime = time.time() - startTime
	print("Total time for one line used", totalTime)



servo = maestro.Controller()

# Servo mapping
startTime = time.time()
servo.setTarget(0,ServoCom1(90))
servo.setTarget(1,ServoCom2(90))
quit()
time.sleep(2)
#quit()

# User Input point

X_start1 = -100		#[mm]
Y_start1 = -180		#[mm]  Max is -120
X_stop1 = 100		#[mm]
Y_stop1 = -180		#[mm] Max is -120

X_start2 = 100	#[mm]
Y_start2 = -180	#[mm]  Max is -120
X_stop2 = 100		#[mm]
Y_stop2 = -150		#[mm] Max is -120

X_start3 = 100		#[mm]
Y_start3 = -150		#[mm]  Max is -120
X_stop3 = -50		#[mm]
Y_stop3 = -150		#[mm] Max is -120

X_start4 = -50		#[mm]
Y_start4 = -150		#[mm]  Max is -120
X_stop4 = -100		#[mm]
Y_stop4 = -180		#[mm] Max is -120

speed = 100;			#[mm/s]

start1 = [X_start1,Y_start1]
stop1 = [X_stop1,Y_stop1]

start2 = [X_start2,Y_start2]
stop2 = [X_stop2,Y_stop2]

start3 = [X_start3,Y_start3]
stop3 = [X_stop3,Y_stop3]

start4 = [X_start4,Y_start4]
stop4 = [X_stop4,Y_stop4]

DriveAng1_1, DriveAng2_1 = CreateLinePath(start1,stop1,speed)
DriveAng1_2, DriveAng2_2 = CreateLinePath(start2,stop2,speed)
DriveAng1_3, DriveAng2_3 = CreateLinePath(start3,stop3,speed)
DriveAng1_4, DriveAng2_4 = CreateLinePath(start4,stop4,speed)

#plt.figure(1)
#plt.plot(plotTime,DriveAng1_1)
#plt.plot(plotTime,DriveAng2)
#plt.show()

#quit()

servo.setTarget(0,ServoCom1(DriveAng1_1[0]))
servo.setTarget(1,ServoCom2(DriveAng2_1[0]))
print(DriveAng1_2[0])
print(DriveAng2_2[0])
time.sleep(2)
#quit()
startTime = time.time()

DrawLine(DriveAng1_1,DriveAng2_1)
DrawLine(DriveAng1_2,DriveAng2_2)
DrawLine(DriveAng1_3,DriveAng2_3)
DrawLine(DriveAng1_4,DriveAng2_4)


totalTime = time.time() - startTime
print("Total time for all path used", totalTime)
servo.close
