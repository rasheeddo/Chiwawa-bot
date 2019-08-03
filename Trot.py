# Test servo with Maestro 

import maestro
import time
import numpy as np
import math as m
import matplotlib.pyplot as plt

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def ServoCom0(deg1):
	return int(map(deg1,0,180,8000,4000))
def ServoCom1(deg2):
	return int(map(deg2,0,180,4000,8000))

def ServoCom2(deg1):
	return int(map(deg1,0,180,8000,4000))
def ServoCom3(deg2):
	return int(map(deg2,0,180,4000,8000))

def ServoCom4(deg1):
	return int(map(deg1,0,180,8000,4000))
def ServoCom5(deg2):
	return int(map(deg2,0,180,4000,8000))

def ServoCom6(deg1):
	return int(map(deg1,0,180,8000,4000))
def ServoCom7(deg2):
	return int(map(deg2,0,180,4000,8000))

def IsMoving():
	while(servo.isMoving(0) or servo.isMoving(1) or servo.isMoving(2) or servo.isMoving(3) or servo.isMoving(4) or servo.isMoving(5) or servo.isMoving(6) or servo.isMoving(7)):
		pass

servo = maestro.Controller()
# Servo mapping
startTime = time.time()
servo.setTarget(0,ServoCom0(90))
servo.setTarget(1,ServoCom1(90))

servo.setTarget(2,ServoCom2(90))
servo.setTarget(3,ServoCom3(90))

servo.setTarget(4,ServoCom4(90))
servo.setTarget(5,ServoCom5(90))

servo.setTarget(6,ServoCom6(90))
servo.setTarget(7,ServoCom7(90))

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
yOffSet = -160 				# max is -120 
T = 100
inc = 1
inc2 = 1
################ front left leg ##################
Af = 30					# max is 50, should be around 10~30, more value makes the robot shake
Bf = -Af
# upper cup
x1FL = np.arange(T/2,-T/2,-inc)
y1FL = np.zeros(shape=np.shape(x1FL))
for i in range(0,len(x1FL)):
	y1FL[i] = (Af*m.sin((m.pi/T)*x1FL[i] + m.pi/2) + yOffSet)
'''
# lower cup
x2FL = np.arange(-T/2,T/2,inc)
y2FL = np.zeros(shape=np.shape(x2FL))
for i in range(0,len(x2FL)):
	y2FL[i] = Bf*m.sin((m.pi/T)*x2FL[i] + m.pi/2) + yOffSet
'''
# staight line drag after
x2FL = np.arange(-T/2,T/2,inc2)
y2FL = np.ones(len(x2FL))*yOffSet

# Combine path
pxFL = np.concatenate((x1FL,x2FL))
pyFL = np.concatenate((y1FL,y2FL))

################ front right leg ##################
# upper cup
x1FR = np.arange(-T/2,T/2,inc)
y1FR = np.zeros(shape=np.shape(x1FR))
for i in range(0,len(x1FR)):
	y1FR[i] = (Af*m.sin((m.pi/T)*x1FR[i] + m.pi/2) + yOffSet)
'''
# lower cup
x2FR = np.arange(T/2,-T/2,-inc)
y2FR = np.zeros(shape=np.shape(x2FR))
for i in range(0,len(x2FR)):
	y2FR[i] = Bf*m.sin((m.pi/T)*x2FR[i] + m.pi/2) + yOffSet
'''
# straight line drag first
x2FR = np.arange(T/2,-T/2,-inc2)
y2FR = np.ones(len(x2FR))*yOffSet

# Combine path
pxFR = np.concatenate((x2FR,x1FR))
pyFR = np.concatenate((y2FR,y1FR))


################ back left leg ##################
Ab = Af
Bb = -Ab
# upper cup
x1BL = np.arange(T/2,-T/2,-inc)
y1BL = np.zeros(shape=np.shape(x1BL))
for i in range(0,len(x1BL)):
	y1BL[i] = (Ab*m.sin((m.pi/T)*x1BL[i] + m.pi/2) + yOffSet)
'''
# lower cup
x2BL = np.arange(-T/2,T/2,inc2)
y2BL = np.zeros(shape=np.shape(x2BL))
for i in range(0,len(x2BL)):
	y2BL[i] = Bb*m.sin((m.pi/T)*x2BL[i] + m.pi/2) + yOffSet
'''
# staight line drag after
x2BL = np.arange(-T/2,T/2,inc2)
y2BL = np.ones(len(x2BL))*(yOffSet)

# Combine path
pxBL = np.concatenate((x2BL,x1BL))
pyBL = np.concatenate((y2BL,y1BL))

################ back right leg ##################

# upper cup
x1BR = np.arange(-T/2,T/2,inc)
y1BR = np.zeros(shape=np.shape(x1BR))
for i in range(0,len(x1BR)):
	y1BR[i] = (Ab*m.sin((m.pi/T)*x1BR[i] + m.pi/2) + yOffSet)
'''
# lower cup
x2BR = np.arange(T/2,-T/2,-inc)
y2BR = np.zeros(shape=np.shape(x2BR))
for i in range(0,len(x2BR)):
	y2BR[i] = Bb*m.sin((m.pi/T)*x2BR[i] + m.pi/2) + yOffSet
'''
# straight line drag first
x2BR = np.arange(T/2,-T/2,-inc2)
y2BR = np.ones(len(x2BR))*(yOffSet)

# Combine path
#pxR = np.concatenate((x1R,x2R))
#pyR = np.concatenate((y1R,y2R))
pxBR = np.concatenate((x1BR,x2BR))		#drag line first then swing
pyBR = np.concatenate((y1BR,y2BR))


## Servo Angle Generation
the1FL =list()
the2FL = list()
the1FR =list()
the2FR = list()
the1BL =list()
the2BL = list()
the1BR =list()
the2BR = list()
plotTime = list()

for i in range(0,len(pxFL)):
	# Front left leg
	RP_FL = m.sqrt((pxFL[i] + RQ/2)**2 + pyFL[i]**2)
	QP_FL = m.sqrt((pxFL[i] - RQ/2)**2 + pyFL[i]**2)

	gam1_FL = m.atan(( pxFL[i] + RQ/2 )/abs(pyFL[i]))
	gam2_FL = m.atan(( pxFL[i] - RQ/2 )/abs(pyFL[i]))

	alp1_FL = m.acos((AP**2 - RP_FL**2 - RA**2)/(-2*RP_FL*RA))
	alp2_FL = m.acos((BP**2 - QP_FL**2 - QB**2)/(-2*QP_FL*QB))

	the1FL.append((alp1_FL-gam1_FL)*180/m.pi)
	the2FL.append((alp2_FL+gam2_FL)*180/m.pi)
	
	# Front right leg
	RP_FR = m.sqrt((pxFR[i] + RQ/2)**2 + pyFR[i]**2)
	QP_FR = m.sqrt((pxFR[i] - RQ/2)**2 + pyFR[i]**2)

	gam1_FR = m.atan(( pxFR[i] + RQ/2 )/abs(pyFR[i]))
	gam2_FR = m.atan(( pxFR[i] - RQ/2 )/abs(pyFR[i]))

	alp1_FR = m.acos((AP**2 - RP_FR**2 - RA**2)/(-2*RP_FR*RA))
	alp2_FR = m.acos((BP**2 - QP_FR**2 - QB**2)/(-2*QP_FR*QB))

	the1FR.append((alp1_FR-gam1_FR)*180/m.pi)
	the2FR.append((alp2_FR+gam2_FR)*180/m.pi)
	
	# Back left leg
	RP_BL = m.sqrt((pxBL[i] + RQ/2)**2 + pyBL[i]**2)
	QP_BL = m.sqrt((pxBL[i] - RQ/2)**2 + pyBL[i]**2)

	gam1_BL = m.atan(( pxBL[i] + RQ/2 )/abs(pyBL[i]))
	gam2_BL = m.atan(( pxBL[i] - RQ/2 )/abs(pyBL[i]))

	alp1_BL = m.acos((AP**2 - RP_BL**2 - RA**2)/(-2*RP_BL*RA))
	alp2_BL = m.acos((BP**2 - QP_BL**2 - QB**2)/(-2*QP_BL*QB))

	the1BL.append((alp1_BL-gam1_BL)*180/m.pi)
	the2BL.append((alp2_BL+gam2_BL)*180/m.pi)

	# Back right leg
	RP_BR = m.sqrt((pxBR[i] + RQ/2)**2 + pyBR[i]**2)
	QP_BR = m.sqrt((pxBR[i] - RQ/2)**2 + pyBR[i]**2)

	gam1_BR = m.atan(( pxBR[i] + RQ/2 )/abs(pyBR[i]))
	gam2_BR = m.atan(( pxBR[i] - RQ/2 )/abs(pyBR[i]))

	alp1_BR = m.acos((AP**2 - RP_BR**2 - RA**2)/(-2*RP_BR*RA))
	alp2_BR = m.acos((BP**2 - QP_BR**2 - QB**2)/(-2*QP_BR*QB))

	the1BR.append((alp1_BR-gam1_BR)*180/m.pi)
	the2BR.append((alp2_BR+gam2_BR)*180/m.pi)
	
	plotTime.append(i)

#plt.figure(1)
#plt.scatter(pxFL,pyFL)
#plt.plot(plotTime,the2)
#plt.show()

servo.setTarget(0,ServoCom0(the1FL[0]))
servo.setTarget(1,ServoCom1(the2FL[0]))

servo.setTarget(2,ServoCom2(the1BL[0]))
servo.setTarget(3,ServoCom3(the2BL[0]))

servo.setTarget(4,ServoCom0(the1FR[0]))
servo.setTarget(5,ServoCom1(the2FR[0]))
servo.setTarget(6,ServoCom2(the1BR[0]))
servo.setTarget(7,ServoCom3(the2BR[0]))

diff = list()
accum = 0
for i in range(0,len(the1FL)):

	if i < (len(the1FL)-1):
		delta = abs(the1FL[i+1] - the1FL[i])
		diff.append(delta)
	else:
		MaxDelta = max(diff)

print(the1FL)
print("max", MaxDelta)
minDelayTime = 0.002*MaxDelta;		#0.12sec/60deg = 0.002sec/deg  HPS A700 servo speed, with the change of MaxDelta, 
									# minDelayTime is the minimum time to travel from one angle to one angle
print("minDelayTime", minDelayTime)

#print(pxFL[0])
#print(pyFL[0])
#print(pxBL[0])
#print(pyBL[0])

time.sleep(2)

samplingTime = 0.005     # the delay should not be lower than 0.005 sec
#samplingTime = minDelayTime + 0.002  #add more 2ms 
print(len(the1FL))
T = time.time()
while True:
	for j in range(0,len(the1FL)):

		T=T+samplingTime
		startTime = time.time()
		#print("the1",the1[j])
		#print("the2",the2[j])
		#print("------------")
		servo.setTarget(0,ServoCom0(the1FL[j]))
		servo.setTarget(1,ServoCom1(the2FL[j]))
		servo.setTarget(6,ServoCom6(the1BR[j]))
		servo.setTarget(7,ServoCom7(the2BR[j]))

		servo.setTarget(2,ServoCom2(the1BL[j]))
		servo.setTarget(3,ServoCom3(the2BL[j]))
		servo.setTarget(4,ServoCom4(the1FR[j]))
		servo.setTarget(5,ServoCom5(the2FR[j]))

		#IsMoving()

		time.sleep(max(0,T-time.time()))     #max is needed in Windows due to


	#time.sleep(1)

	

#print("input_pos=%f" %pos)
#print("servo_pos=%f" %servo_pos)
#print("-----------------------------")

servo.close
