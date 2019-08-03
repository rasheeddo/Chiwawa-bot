# Test servo with Maestro 

import maestro
import time
import numpy as np
import math as m
import matplotlib.pyplot as plt

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def ServoCom1(deg1):
	return int(map(deg1,0,180,8000,4000))
def ServoCom2(deg2):
	return int(map(deg2,0,180,4000,8000))

servo = maestro.Controller()
# Servo mapping
startTime = time.time()
servo.setTarget(0,ServoCom1(90))
servo.setTarget(1,ServoCom2(90))
#quit()
time.sleep(2)
#quit()
## Parameters
L1 = 74
L2 = 180
RA = L1
QB = L1
AP = L2
BP = L2
RQ = 40

## Path Generation
yOffSet = -180
T = 200
inc = 1
# upper cup
A = 30
x1 = np.arange(T/2,-T/2,-inc)
y1 = np.zeros(shape=np.shape(x1))
for i in range(0,len(x1)):
	y1[i] = (A*m.sin((m.pi/T)*x1[i] + m.pi/2) + yOffSet)
# lower cup
'''
B = -A
x2 = np.arange(-T/2,T/2,inc)
y2 = np.zeros(shape=np.shape(x2))
for i in range(0,len(x2)):
	y2[i] = B*m.sin((m.pi/T)*x2[i] + m.pi/2) + yOffSet
'''
# staight line drag after
x2 = np.arange(-T/2,T/2,inc)
y2 = np.ones(len(x2))*yOffSet
# Combine path
px = np.concatenate((x1,x2))
py = np.concatenate((y1,y2))

## Servo Angle Generation
the1 =list()
the2 = list()
plotTime = list()
for i in range(0,len(px)):
	RP = m.sqrt((px[i] + RQ/2)**2 + py[i]**2)
	QP = m.sqrt((px[i] - RQ/2)**2 + py[i]**2)

	gam1 = m.atan(( px[i] + RQ/2 )/abs(py[i]))
	gam2 = m.atan(( px[i] - RQ/2 )/abs(py[i]))

	alp1 = m.acos((AP**2 - RP**2 - RA**2)/(-2*RP*RA))
	alp2 = m.acos((BP**2 - QP**2 - QB**2)/(-2*QP*QB))

	the1.append((alp1-gam1)*180/m.pi)
	the2.append((alp2+gam2)*180/m.pi)
	plotTime.append(i)

#plt.figure(1)
#plt.plot(plotTime,the1)
#plt.plot(plotTime,the2)
#plt.show()

servo.setTarget(0,ServoCom1(the1[0]))
servo.setTarget(1,ServoCom2(the2[0]))
time.sleep(2)

samplingTime = 0.005
T = time.time()
while True:
	for j in range(0,len(the1)):

		T=T+samplingTime
		startTime = time.time()
		#print("the1",the1[j])
		#print("the2",the2[j])
		#print("------------")
		servo.setTarget(0,ServoCom1(the1[j]))
		servo.setTarget(1,ServoCom2(the2[j]))

		time.sleep(max(0,T-time.time()))     #max is needed in Windows due to


#print("input_pos=%f" %pos)
#print("servo_pos=%f" %servo_pos)
#print("-----------------------------")

servo.close
