
import maestro
import time
import numpy as np
import math as m
import matplotlib.pyplot as plt

class Chiwawa:

	def __init__(self):
		self.servo = maestro.Controller()

		# Leg parameters
		self.L1 = 74
		self.L2 = 180
		self.RA = self.L1
		self.QB = self.L1
		self.AP = self.L2
		self.BP = self.L2
		self.RQ = 40

		self.PF = 64					# Foot length
		self.BF = self.BP + self.PF
		self.AF = self.BF

		# For walk motion
		self.stepWidth = 100
		self.stepAmp = 70
		self.stepHeight = -200

		### Initialiaze all path

		# Stand By
		self.standBy_X = 0.0
		self.standBy_Y = -250.0
		self.Deg1FL_StandBy, self.Deg2FL_StandBy = self.LeftLegINV(self.standBy_X, self.standBy_Y)	# FL
		self.Deg1BL_StandBy, self.Deg2BL_StandBy = self.LeftLegINV(self.standBy_X, self.standBy_Y)	# BL
		self.Deg1FR_StandBy, self.Deg2FR_StandBy = self.RightLegINV(self.standBy_X, self.standBy_Y)	# FR
		self.Deg1BR_StandBy, self.Deg2BR_StandBy = self.RightLegINV(self.standBy_X, self.standBy_Y)	# BR
		
		# Step
		self.leanOffset = 0.0
		self.StepUp = -190.0
		self.StepDown = -250.0

		self.delayUpDown = 0.1
		self.pauseTimeStep = 0.5	

		self.Deg1L_StepUp, self.Deg2L_StepUp = self.LeftLegINV(self.leanOffset, self.StepUp)	
		self.Deg1R_StepUp, self.Deg2R_StepUp = self.RightLegINV(-self.leanOffset, self.StepUp)
		self.Deg1L_StepDown, self.Deg2L_StepDown = self.LeftLegINV(self.leanOffset, self.StepDown)
		self.Deg1R_StepDown, self.Deg2R_StepDown = self.RightLegINV(-self.leanOffset, self.StepDown)


		# Lean Back
		self.leanBack = -100.0
		self.Deg1L_LeanBack, self.Deg2L_LeanBack = self.LegINV(self.leanBack,-180.0)
		self.Deg1R_LeanBack, self.Deg2R_LeanBack = self.LegINV(-self.leanBack,-180.0)	

		# Trot
		self.yOffSet_Trot = -230.0 			
		self.T_Trot = 100.0
		self.A_Trot = 50.0	
		self.samplingTimeTrot = 0.002   #0.002  
				
		self.Deg1FL, self.Deg2FL, self.Deg1BL, self.Deg2BL =  self.PathGenLeft(self.yOffSet_Trot, self.T_Trot, self.A_Trot)
		self.Deg1FR, self.Deg2FR, self.Deg1BR, self.Deg2BR =  self.PathGenRight(self.yOffSet_Trot, self.T_Trot, self.A_Trot)

		# Trot Back   Still need to tune....
		self.yOffSet_TrotBack = -230.0 			
		self.T_TrotBack = 100.0
		self.A_TrotBack = 30.0	
		self.samplingTimeTrotBack = 0.002     
																									# When walk backward, left leg side gonna be right leg side
		self.Deg1FL_TrotBack, self.Deg2FL_TrotBack, self.Deg1BL_TrotBack, self.Deg2BL_TrotBack =  self.PathGenRightBack(self.yOffSet_TrotBack, self.T_TrotBack, self.A_TrotBack)
		self.Deg1FR_TrotBack, self.Deg2FR_TrotBack, self.Deg1BR_TrotBack, self.Deg2BR_TrotBack =  self.PathGenLeftBack(self.yOffSet_TrotBack, self.T_TrotBack, self.A_TrotBack)

		# Turn Right
		self.yOffSet_TurnRight = -230.0				
		self.TL_TurnRight = 100.0
		self.TR_TurnRight = 50.0
		self.A_TurnRight = 60.0
		self.samplingTimeTurnRight = 0.002     
		
		self.Deg1FL_TurnRight, self.Deg2FL_TurnRight, self.Deg1BL_TurnRight, self.Deg2BL_TurnRight =  self.PathGenLeft(self.yOffSet_TurnRight, self.TL_TurnRight, self.A_TurnRight)
		self.Deg1FR_TurnRight, self.Deg2FR_TurnRight, self.Deg1BR_TurnRight, self.Deg2BR_TurnRight =  self.PathGenRight(self.yOffSet_TurnRight, self.TR_TurnRight, self.A_TurnRight)

		# Turn Left
		self.yOffSet_TurnLeft = -230.0				
		self.TL_TurnLeft = 50.0
		self.TR_TurnLeft = 100.0
		self.A_TurnLeft = 50.0
		self.samplingTimeTurnLeft = 0.002  
			
		self.Deg1FL_TurnLeft, self.Deg2FL_TurnLeft, self.Deg1BL_TurnLeft, self.Deg2BL_TurnLeft =  self.PathGenLeft(self.yOffSet_TurnLeft, self.TL_TurnLeft, self.A_TurnLeft)
		self.Deg1FR_TurnLeft, self.Deg2FR_TurnLeft, self.Deg1BR_TurnLeft, self.Deg2BR_TurnLeft =  self.PathGenRight(self.yOffSet_TurnLeft, self.TR_TurnLeft, self.A_TurnLeft)


	def map(self,val, in_min, in_max, out_min, out_max):

		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def ServoComLeft(self,deg1):
		return int(self.map(deg1,0,180,8000,4000))

	def ServoComRight(self,deg2):
		return int(self.map(deg2,0,180,4000,8000))

	def StandBy(self):

		# FL
		self.servo.setTarget(0,self.ServoComLeft(self.Deg1FL_StandBy))
		self.servo.setTarget(1,self.ServoComRight(self.Deg2FL_StandBy))
		# BL
		self.servo.setTarget(2,self.ServoComLeft(self.Deg1BL_StandBy))
		self.servo.setTarget(3,self.ServoComRight(self.Deg2BL_StandBy))
		# FR
		self.servo.setTarget(4,self.ServoComLeft(self.Deg1FR_StandBy))
		self.servo.setTarget(5,self.ServoComRight(self.Deg2FR_StandBy))
		# BR
		self.servo.setTarget(6,self.ServoComLeft(self.Deg1BR_StandBy))
		self.servo.setTarget(7,self.ServoComRight(self.Deg2BR_StandBy))

		#time.sleep(2)


	def IsMoving(self):
		while(self.servo.isMoving(0) or self.servo.isMoving(1) or self.servo.isMoving(2) or self.servo.isMoving(3) or 
			self.servo.isMoving(4) or self.servo.isMoving(5) or self.servo.isMoving(6) or self.servo.isMoving(7)):
			pass

	def LegINV(self,px,py):

		RP = m.sqrt((px + self.RQ/2)**2 + py**2)
		QP = m.sqrt((px - self.RQ/2)**2 + py**2)


		gam1 = m.atan(( px + self.RQ/2 )/abs(py))
		gam2 = m.atan(( px - self.RQ/2 )/abs(py))


		alp1 = m.acos((self.AP**2 - RP**2 - self.RA**2)/(-2*RP*self.RA))
		alp2 = m.acos((self.BP**2 - QP**2 - self.QB**2)/(-2*QP*self.QB))


		the1 = (alp1-gam1)*180/m.pi
		the2 = (alp2+gam2)*180/m.pi

		return the1,the2

	def LeftLegINV(self,fx,fy):
		QF_L = m.sqrt( (fx - (self.RQ/2))**2 + fy**2)
		bet2_L = m.atan( (fx-(self.RQ/2)) / abs(fy) )

		phi2_L = m.acos( (self.BF**2 - self.QB**2 - QF_L**2)/(-2*self.QB*QF_L))

		the2_L = phi2_L + bet2_L

		bx_L = self.QB*m.sin(the2_L) + self.RQ/2
		by_L = -self.QB*m.cos(the2_L)

		vectBF = np.array([(fx - bx_L), (fy - by_L)])
		unitBF = vectBF/abs(self.BF)
		unitBP = unitBF
		vectBP = unitBP*self.BP
		px_L = vectBP[0] + bx_L
		py_L = vectBP[1] + by_L

		RP_L = m.sqrt(( px_L + self.RQ/2 )**2 + py_L**2)
		gam1_L = m.atan(( px_L + self.RQ/2 )/abs(py_L))
		alp1_L = m.acos( (self.AP**2 - RP_L**2 - self.RA**2)/(-2*RP_L*self.RA))

		the1_L = alp1_L - gam1_L

		ax_L = -self.RA*m.sin(the1_L) - self.RQ/2
		ay_L = -self.RA*m.cos(the1_L)

		deg1_L = the1_L*180/m.pi
		deg2_L = the2_L*180/m.pi

		return deg1_L, deg2_L

	def RightLegINV(self,fx,fy):
		RF_R = m.sqrt( (fx + (self.RQ/2))**2 + fy**2)
		bet1_R = m.atan( (fx +(self.RQ/2)) /abs(fy))
		phi1_R = m.acos( (self.AF**2 - self.RA**2 - RF_R**2)/(-2*self.RA*RF_R))
		the1_R = phi1_R - bet1_R

		ax_R = -self.RA*m.sin(the1_R) - self.RQ/2
		ay_R = -self.RA*m.cos(the1_R)

		vectAF = np.array([(fx - ax_R), (fy - ay_R)])
		unitAF = vectAF/abs(self.AF)
		unitAP = unitAF
		vectAP = unitAP*self.AP
		px_R = vectAP[0] + ax_R
		py_R = vectAP[1] + ay_R

		QP_R = m.sqrt(( px_R - self.RQ/2 )**2 + py_R**2)
		gam2_R = m.atan(( px_R - self.RQ/2 )/abs(py_R))
		alp2_R = m.acos( (self.BP**2 - QP_R**2 - self.QB**2)/(-2*QP_R*self.QB))

		the2_R = alp2_R + gam2_R

		bx_R = self.QB*m.sin(the2_R) + self.RQ/2
		by_R = -self.QB*m.cos(the2_R)

		deg1_R = the1_R*180/m.pi
		deg2_R = the2_R*180/m.pi

		return deg1_R, deg2_R



	def Kill(self):

		self.servo.close()

	def Step(self,Trig):

		while Trig:

			# FL
			self.servo.setTarget(0,self.ServoComLeft(self.Deg1L_StepUp))
			self.servo.setTarget(1,self.ServoComRight(self.Deg2L_StepUp))
			# BR
			self.servo.setTarget(6,self.ServoComLeft(self.Deg1R_StepUp))
			self.servo.setTarget(7,self.ServoComRight(self.Deg2R_StepUp))
			time.sleep(self.delayUpDown)   
			# FL
			self.servo.setTarget(0,self.ServoComLeft(self.Deg1L_StepDown))
			self.servo.setTarget(1,self.ServoComRight(self.Deg2L_StepDown))
			# BR
			self.servo.setTarget(6,self.ServoComLeft(self.Deg1R_StepDown))
			self.servo.setTarget(7,self.ServoComRight(self.Deg2R_StepDown))
			time.sleep(self.pauseTimeStep)
			
			# BL
			self.servo.setTarget(2,self.ServoComLeft(self.Deg1L_StepUp))
			self.servo.setTarget(3,self.ServoComRight(self.Deg2L_StepUp))
			# FR
			self.servo.setTarget(4,self.ServoComLeft(self.Deg1R_StepUp))
			self.servo.setTarget(5,self.ServoComRight(self.Deg2R_StepUp))
			time.sleep(self.delayUpDown)   

			# BL
			self.servo.setTarget(2,self.ServoComLeft(self.Deg1L_StepDown))
			self.servo.setTarget(3,self.ServoComRight(self.Deg2L_StepDown))
			# FR
			self.servo.setTarget(4,self.ServoComLeft(self.Deg1R_StepDown))
			self.servo.setTarget(5,self.ServoComRight(self.Deg2R_StepDown))
			time.sleep(self.pauseTimeStep)  

	def StepJoy(self):

		# FL
		self.servo.setTarget(0,self.ServoComLeft(self.Deg1L_StepUp))
		self.servo.setTarget(1,self.ServoComRight(self.Deg2L_StepUp))
		# BR
		self.servo.setTarget(6,self.ServoComLeft(self.Deg1R_StepUp))
		self.servo.setTarget(7,self.ServoComRight(self.Deg2R_StepUp))
		time.sleep(self.delayUpDown)   
		# FL
		self.servo.setTarget(0,self.ServoComLeft(self.Deg1L_StepDown))
		self.servo.setTarget(1,self.ServoComRight(self.Deg2L_StepDown))
		# BR
		self.servo.setTarget(6,self.ServoComLeft(self.Deg1R_StepDown))
		self.servo.setTarget(7,self.ServoComRight(self.Deg2R_StepDown))
		time.sleep(self.pauseTimeStep)
		
		# BL
		self.servo.setTarget(2,self.ServoComLeft(self.Deg1L_StepUp))
		self.servo.setTarget(3,self.ServoComRight(self.Deg2L_StepUp))
		# FR
		self.servo.setTarget(4,self.ServoComLeft(self.Deg1R_StepUp))
		self.servo.setTarget(5,self.ServoComRight(self.Deg2R_StepUp))
		time.sleep(self.delayUpDown)   

		# BL
		self.servo.setTarget(2,self.ServoComLeft(self.Deg1L_StepDown))
		self.servo.setTarget(3,self.ServoComRight(self.Deg2L_StepDown))
		# FR
		self.servo.setTarget(4,self.ServoComLeft(self.Deg1R_StepDown))
		self.servo.setTarget(5,self.ServoComRight(self.Deg2R_StepDown))
		time.sleep(self.pauseTimeStep)  


	def PitchBody(self,gain):

		# Lean
		lean = self.standBy_X + gain  # gain must be -x_max to x_max
		Deg1L_LeanForth, Deg2L_LeanForth = self.LeftLegINV(lean, self.standBy_Y)	
		Deg1R_LeanForth, Deg2R_LeanForth = self.RightLegINV(-lean, self.standBy_Y)
	
		# FL
		self.servo.setTarget(0,self.ServoComLeft(Deg1L_LeanForth))
		self.servo.setTarget(1,self.ServoComRight(Deg2L_LeanForth))
		# BL
		self.servo.setTarget(2,self.ServoComLeft(Deg1L_LeanForth))
		self.servo.setTarget(3,self.ServoComRight(Deg2L_LeanForth))
		# FR
		self.servo.setTarget(4,self.ServoComLeft(Deg1R_LeanForth))
		self.servo.setTarget(5,self.ServoComRight(Deg1R_LeanForth))
		# BR
		self.servo.setTarget(6,self.ServoComLeft(Deg1R_LeanForth))
		self.servo.setTarget(7,self.ServoComRight(Deg1R_LeanForth))


	def PathGenLeft(self,yOffSet,T,A):

		inc = T/100.0

		################ front left leg ##################

		# upper cup
		x1FL = np.arange(T/2,-T/2,-inc)
		y1FL = np.zeros(shape=np.shape(x1FL))
		for i in range(0,len(x1FL)):
			y1FL[i] = (A*m.sin((m.pi/T)*x1FL[i] + m.pi/2) + yOffSet)
		'''
		# lower cup
		x2FL = np.arange(-T/2,T/2,inc)
		y2FL = np.zeros(shape=np.shape(x2FL))
		for i in range(0,len(x2FL)):
			y2FL[i] = Bf*m.sin((m.pi/T)*x2FL[i] + m.pi/2) + yOffSet
		'''
		# staight line drag after
		x2FL = np.arange(-T/2,T/2,inc)
		y2FL = np.ones(len(x2FL))*yOffSet

		# Combine path
		fxFL = np.concatenate((x1FL,x2FL))
		fyFL = np.concatenate((y1FL,y2FL))


		################ back left leg ##################
		# upper cup
		x1BL = np.arange(T/2,-T/2,-inc)
		y1BL = np.zeros(shape=np.shape(x1BL))
		for i in range(0,len(x1BL)):
			y1BL[i] = (A*m.sin((m.pi/T)*x1BL[i] + m.pi/2) + yOffSet)
		'''
		# lower cup
		x2BL = np.arange(-T/2,T/2,inc2)
		y2BL = np.zeros(shape=np.shape(x2BL))
		for i in range(0,len(x2BL)):
			y2BL[i] = Bb*m.sin((m.pi/T)*x2BL[i] + m.pi/2) + yOffSet
		'''
		# staight line drag after
		x2BL = np.arange(-T/2,T/2,inc)
		y2BL = np.ones(len(x2BL))*(yOffSet)

		# Combine path
		fxBL = np.concatenate((x2BL,x1BL))
		fyBL = np.concatenate((y2BL,y1BL))


		## Servo Angle Generation
		the1FL =list()
		the2FL = list()
		the1BL =list()
		the2BL = list()
		plotTime = list()

		for i in range(0,len(fxFL)):

			# Front left leg
			the1fl, the2fl = self.LeftLegINV(fxFL[i],fyFL[i])

			the1FL.append(the1fl)
			the2FL.append(the2fl)
			
			# Back left leg
			the1bl, the2bl = self.LeftLegINV(fxBL[i],fyBL[i])

			the1BL.append(the1bl)
			the2BL.append(the2bl)

			
			plotTime.append(i)

		return the1FL, the2FL,  the1BL, the2BL, 

	def PathGenRight(self,yOffSet,T,A,):

		inc = T/100.0

		################ front right leg ##################
		# upper cup
		x1FR = np.arange(-T/2,T/2,inc)
		y1FR = np.zeros(shape=np.shape(x1FR))
		for i in range(0,len(x1FR)):
			y1FR[i] = (A*m.sin((m.pi/T)*x1FR[i] + m.pi/2) + yOffSet)
		'''
		# lower cup
		x2FR = np.arange(T/2,-T/2,-inc)
		y2FR = np.zeros(shape=np.shape(x2FR))
		for i in range(0,len(x2FR)):
			y2FR[i] = Bf*m.sin((m.pi/T)*x2FR[i] + m.pi/2) + yOffSet
		'''
		# straight line drag first
		x2FR = np.arange(T/2,-T/2,-inc)
		y2FR = np.ones(len(x2FR))*yOffSet

		# Combine path
		fxFR = np.concatenate((x2FR,x1FR))
		fyFR = np.concatenate((y2FR,y1FR))


		################ back right leg ##################

		# upper cup
		x1BR = np.arange(-T/2,T/2,inc)
		y1BR = np.zeros(shape=np.shape(x1BR))
		for i in range(0,len(x1BR)):
			y1BR[i] = (A*m.sin((m.pi/T)*x1BR[i] + m.pi/2) + yOffSet)
		'''
		# lower cup
		x2BR = np.arange(T/2,-T/2,-inc)
		y2BR = np.zeros(shape=np.shape(x2BR))
		for i in range(0,len(x2BR)):
			y2BR[i] = Bb*m.sin((m.pi/T)*x2BR[i] + m.pi/2) + yOffSet
		'''
		# straight line drag first
		x2BR = np.arange(T/2,-T/2,-inc)
		y2BR = np.ones(len(x2BR))*(yOffSet)

		# Combine path
		#pxR = np.concatenate((x1R,x2R))
		#pyR = np.concatenate((y1R,y2R))
		fxBR = np.concatenate((x1BR,x2BR))		#drag line first then swing
		fyBR = np.concatenate((y1BR,y2BR))


		## Servo Angle Generation

		the1FR =list()
		the2FR = list()
		the1BR =list()
		the2BR = list()
		plotTime = list()

		for i in range(0,len(fxFR)):

			# Front right leg

			the1fr, the2fr = self.RightLegINV(fxFR[i],fyFR[i])

			the1FR.append(the1fr)
			the2FR.append(the2fr)

			# Back right leg

			the1br, the2br = self.RightLegINV(fxBR[i],fyBR[i])

			the1BR.append(the1br)
			the2BR.append(the2br)
			
			plotTime.append(i)

		return  the1FR, the2FR,  the1BR, the2BR


	def PathGenLeftBack(self,yOffSet,T,A):

		# When walk backward, left leg side gonna be right leg side

		inc = T/100.0

		################ front left leg ##################

		# upper cup
		x1FL = np.arange(T/2,-T/2,-inc)
		y1FL = np.zeros(shape=np.shape(x1FL))
		for i in range(0,len(x1FL)):
			y1FL[i] = (A*m.sin((m.pi/T)*x1FL[i] + m.pi/2) + yOffSet)
		'''
		# lower cup
		x2FL = np.arange(-T/2,T/2,inc)
		y2FL = np.zeros(shape=np.shape(x2FL))
		for i in range(0,len(x2FL)):
			y2FL[i] = Bf*m.sin((m.pi/T)*x2FL[i] + m.pi/2) + yOffSet
		'''
		# staight line drag after
		x2FL = np.arange(-T/2,T/2,inc)
		y2FL = np.ones(len(x2FL))*yOffSet

		# Combine path
		fxFL = np.concatenate((x1FL,x2FL))
		fyFL = np.concatenate((y1FL,y2FL))


		################ back left leg ##################
		# upper cup
		x1BL = np.arange(T/2,-T/2,-inc)
		y1BL = np.zeros(shape=np.shape(x1BL))
		for i in range(0,len(x1BL)):
			y1BL[i] = (A*m.sin((m.pi/T)*x1BL[i] + m.pi/2) + yOffSet)
		'''
		# lower cup
		x2BL = np.arange(-T/2,T/2,inc2)
		y2BL = np.zeros(shape=np.shape(x2BL))
		for i in range(0,len(x2BL)):
			y2BL[i] = Bb*m.sin((m.pi/T)*x2BL[i] + m.pi/2) + yOffSet
		'''
		# staight line drag after
		x2BL = np.arange(-T/2,T/2,inc)
		y2BL = np.ones(len(x2BL))*(yOffSet)

		# Combine path
		fxBL = np.concatenate((x2BL,x1BL))
		fyBL = np.concatenate((y2BL,y1BL))


		## Servo Angle Generation
		the1FL =list()
		the2FL = list()
		the1BL =list()
		the2BL = list()
		plotTime = list()

		for i in range(0,len(fxFL)):

			# Front left leg
			the1fl, the2fl = self.RightLegINV(fxFL[i],fyFL[i])

			the1FL.append(the1fl)
			the2FL.append(the2fl)
			
			# Back left leg
			the1bl, the2bl = self.RightLegINV(fxBL[i],fyBL[i])

			the1BL.append(the1bl)
			the2BL.append(the2bl)

			
			plotTime.append(i)

		return the1FL, the2FL,  the1BL, the2BL, 

	def PathGenRightBack(self,yOffSet,T,A,):

		# When walk backward, left leg side gonna be right leg side

		inc = T/100.0

		################ front right leg ##################
		# upper cup
		x1FR = np.arange(-T/2,T/2,inc)
		y1FR = np.zeros(shape=np.shape(x1FR))
		for i in range(0,len(x1FR)):
			y1FR[i] = (A*m.sin((m.pi/T)*x1FR[i] + m.pi/2) + yOffSet)
		'''
		# lower cup
		x2FR = np.arange(T/2,-T/2,-inc)
		y2FR = np.zeros(shape=np.shape(x2FR))
		for i in range(0,len(x2FR)):
			y2FR[i] = Bf*m.sin((m.pi/T)*x2FR[i] + m.pi/2) + yOffSet
		'''
		# straight line drag first
		x2FR = np.arange(T/2,-T/2,-inc)
		y2FR = np.ones(len(x2FR))*yOffSet

		# Combine path
		fxFR = np.concatenate((x2FR,x1FR))
		fyFR = np.concatenate((y2FR,y1FR))


		################ back right leg ##################

		# upper cup
		x1BR = np.arange(-T/2,T/2,inc)
		y1BR = np.zeros(shape=np.shape(x1BR))
		for i in range(0,len(x1BR)):
			y1BR[i] = (A*m.sin((m.pi/T)*x1BR[i] + m.pi/2) + yOffSet)
		'''
		# lower cup
		x2BR = np.arange(T/2,-T/2,-inc)
		y2BR = np.zeros(shape=np.shape(x2BR))
		for i in range(0,len(x2BR)):
			y2BR[i] = Bb*m.sin((m.pi/T)*x2BR[i] + m.pi/2) + yOffSet
		'''
		# straight line drag first
		x2BR = np.arange(T/2,-T/2,-inc)
		y2BR = np.ones(len(x2BR))*(yOffSet)

		# Combine path
		#pxR = np.concatenate((x1R,x2R))
		#pyR = np.concatenate((y1R,y2R))
		fxBR = np.concatenate((x1BR,x2BR))		#drag line first then swing
		fyBR = np.concatenate((y1BR,y2BR))


		## Servo Angle Generation

		the1FR =list()
		the2FR = list()
		the1BR =list()
		the2BR = list()
		plotTime = list()

		for i in range(0,len(fxFR)):

			# Front right leg

			the1fr, the2fr = self.LeftLegINV(fxFR[i],fyFR[i])

			the1FR.append(the1fr)
			the2FR.append(the2fr)

			# Back right leg

			the1br, the2br = self.LeftLegINV(fxBR[i],fyBR[i])

			the1BR.append(the1br)
			the2BR.append(the2br)
			
			plotTime.append(i)

		return  the1FR, the2FR,  the1BR, the2BR

	def Trot(self,Trig):

		t = time.time()
		while Trig:

			for j in range(0,len(self.Deg1FL)):

				t = t + self.samplingTimeTrot
				startTime = time.time()
				self.servo.setTarget(0,self.ServoComLeft(self.Deg1FL[j]))
				self.servo.setTarget(1,self.ServoComRight(self.Deg2FL[j]))
				self.servo.setTarget(6,self.ServoComLeft(self.Deg1BR[j]))
				self.servo.setTarget(7,self.ServoComRight(self.Deg2BR[j]))

				self.servo.setTarget(2,self.ServoComLeft(self.Deg1BL[j]))
				self.servo.setTarget(3,self.ServoComRight(self.Deg2BL[j]))
				self.servo.setTarget(4,self.ServoComLeft(self.Deg1FR[j]))
				self.servo.setTarget(5,self.ServoComRight(self.Deg2FR[j]))

				timeDelay = max(0,t-time.time())
				print("timeDelay %.7f" %timeDelay)
				time.sleep(timeDelay)     #max is needed in Windows due to

	def TrotJoy(self):

		t = time.time()

		for j in range(0,len(self.Deg1FL)):

			t = t + self.samplingTimeTrot
			startTime = time.time()
			self.servo.setTarget(0,self.ServoComLeft(self.Deg1FL[j]))
			self.servo.setTarget(1,self.ServoComRight(self.Deg2FL[j]))
			self.servo.setTarget(6,self.ServoComLeft(self.Deg1BR[j]))
			self.servo.setTarget(7,self.ServoComRight(self.Deg2BR[j]))

			self.servo.setTarget(2,self.ServoComLeft(self.Deg1BL[j]))
			self.servo.setTarget(3,self.ServoComRight(self.Deg2BL[j]))
			self.servo.setTarget(4,self.ServoComLeft(self.Deg1FR[j]))
			self.servo.setTarget(5,self.ServoComRight(self.Deg2FR[j]))

			timeDelay = max(0,t-time.time())
			print("timeDelay %.7f" %timeDelay)
			time.sleep(timeDelay)     #max is needed in Windows due to

	def TrotBack(self,Trig):

		t = time.time()
		while Trig:

			for j in range(0,len(self.Deg1FL_TrotBack)):

				t = t + self.samplingTimeTrotBack
				startTime = time.time()
				self.servo.setTarget(0,self.ServoComLeft(self.Deg1FL_TrotBack[j]))
				self.servo.setTarget(1,self.ServoComRight(self.Deg2FL_TrotBack[j]))
				self.servo.setTarget(6,self.ServoComLeft(self.Deg1BR_TrotBack[j]))
				self.servo.setTarget(7,self.ServoComRight(self.Deg2BR_TrotBack[j]))

				self.servo.setTarget(2,self.ServoComLeft(self.Deg1BL_TrotBack[j]))
				self.servo.setTarget(3,self.ServoComRight(self.Deg2BL_TrotBack[j]))
				self.servo.setTarget(4,self.ServoComLeft(self.Deg1FR_TrotBack[j]))
				self.servo.setTarget(5,self.ServoComRight(self.Deg2FR_TrotBack[j]))

				timeDelay = max(0,t-time.time())
				print("timeDelay %.7f" %timeDelay)
				time.sleep(timeDelay)     #max is needed in Windows due to

	def TrotBackJoy(self):

		t = time.time()

		for j in range(0,len(self.Deg1FL_TrotBack)):

			t = t + self.samplingTimeTrotBack
			startTime = time.time()
			self.servo.setTarget(0,self.ServoComLeft(self.Deg1FL_TrotBack[j]))
			self.servo.setTarget(1,self.ServoComRight(self.Deg2FL_TrotBack[j]))
			self.servo.setTarget(6,self.ServoComLeft(self.Deg1BR_TrotBack[j]))
			self.servo.setTarget(7,self.ServoComRight(self.Deg2BR_TrotBack[j]))

			self.servo.setTarget(2,self.ServoComLeft(self.Deg1BL_TrotBack[j]))
			self.servo.setTarget(3,self.ServoComRight(self.Deg2BL_TrotBack[j]))
			self.servo.setTarget(4,self.ServoComLeft(self.Deg1FR_TrotBack[j]))
			self.servo.setTarget(5,self.ServoComRight(self.Deg2FR_TrotBack[j]))

			timeDelay = max(0,t-time.time())
			print("timeDelay %.7f" %timeDelay)
			time.sleep(timeDelay)     #max is needed in Windows due to

	def TurnRight(self,Trig):

		t = time.time()
		while Trig:
			for j in range(0,len(self.Deg1FL)):

				t = t + self.samplingTimeTurnRight
				startTime = time.time()
				#print("the1",the1[j])
				#print("the2",the2[j])
				#print("------------")
				self.servo.setTarget(0,self.ServoComLeft(self.Deg1FL_TurnRight[j]))
				self.servo.setTarget(1,self.ServoComRight(self.Deg2FL_TurnRight[j]))
				self.servo.setTarget(6,self.ServoComLeft(self.Deg1BR_TurnRight[j]))
				self.servo.setTarget(7,self.ServoComRight(self.Deg2BR_TurnRight[j]))

				self.servo.setTarget(2,self.ServoComLeft(self.Deg1BL_TurnRight[j]))
				self.servo.setTarget(3,self.ServoComRight(self.Deg2BL_TurnRight[j]))
				self.servo.setTarget(4,self.ServoComLeft(self.Deg1FR_TurnRight[j]))
				self.servo.setTarget(5,self.ServoComRight(self.Deg2FR_TurnRight[j]))

				#IsMoving()
				#time.sleep(0.005)
				timeDelay = max(0,t-time.time())
				print("timeDelay %.7f" %timeDelay)
				time.sleep(timeDelay)     #max is needed in Windows due 

	def TurnLeft(self,Trig):

		t = time.time()
		while Trig:
			for j in range(0,len(self.Deg1FL)):

				t = t + self.samplingTimeTurnLeft
				startTime = time.time()
				#print("the1",the1[j])
				#print("the2",the2[j])
				#print("------------")
				self.servo.setTarget(0,self.ServoComLeft(self.Deg1FL_TurnLeft[j]))
				self.servo.setTarget(1,self.ServoComRight(self.Deg2FL_TurnLeft[j]))
				self.servo.setTarget(6,self.ServoComLeft(self.Deg1BR_TurnLeft[j]))
				self.servo.setTarget(7,self.ServoComRight(self.Deg2BR_TurnLeft[j]))

				self.servo.setTarget(2,self.ServoComLeft(self.Deg1BL_TurnLeft[j]))
				self.servo.setTarget(3,self.ServoComRight(self.Deg2BL_TurnLeft[j]))
				self.servo.setTarget(4,self.ServoComLeft(self.Deg1FR_TurnLeft[j]))
				self.servo.setTarget(5,self.ServoComRight(self.Deg2FR_TurnLeft[j]))

				#IsMoving()
				#time.sleep(0.005)
				timeDelay = max(0,t-time.time())
				print("timeDelay %.7f" %timeDelay)
				time.sleep(timeDelay)     #max is needed in Windows due to

	def TurnRightJoy(self):

		t = time.time()

		for j in range(0,len(self.Deg1FL)):

			t = t + self.samplingTimeTurnRight
			startTime = time.time()
			#print("the1",the1[j])
			#print("the2",the2[j])
			#print("------------")
			self.servo.setTarget(0,self.ServoComLeft(self.Deg1FL_TurnRight[j]))
			self.servo.setTarget(1,self.ServoComRight(self.Deg2FL_TurnRight[j]))
			self.servo.setTarget(6,self.ServoComLeft(self.Deg1BR_TurnRight[j]))
			self.servo.setTarget(7,self.ServoComRight(self.Deg2BR_TurnRight[j]))

			self.servo.setTarget(2,self.ServoComLeft(self.Deg1BL_TurnRight[j]))
			self.servo.setTarget(3,self.ServoComRight(self.Deg2BL_TurnRight[j]))
			self.servo.setTarget(4,self.ServoComLeft(self.Deg1FR_TurnRight[j]))
			self.servo.setTarget(5,self.ServoComRight(self.Deg2FR_TurnRight[j]))

			#IsMoving()
			#time.sleep(0.005)
			timeDelay = max(0,t-time.time())
			print("timeDelay %.7f" %timeDelay)
			time.sleep(timeDelay)     #max is needed in Windows due 

	def TurnLeftJoy(self):

		t = time.time()

		for j in range(0,len(self.Deg1FL)):

			t = t + self.samplingTimeTurnLeft
			startTime = time.time()
			#print("the1",the1[j])
			#print("the2",the2[j])
			#print("------------")
			self.servo.setTarget(0,self.ServoComLeft(self.Deg1FL_TurnLeft[j]))
			self.servo.setTarget(1,self.ServoComRight(self.Deg2FL_TurnLeft[j]))
			self.servo.setTarget(6,self.ServoComLeft(self.Deg1BR_TurnLeft[j]))
			self.servo.setTarget(7,self.ServoComRight(self.Deg2BR_TurnLeft[j]))

			self.servo.setTarget(2,self.ServoComLeft(self.Deg1BL_TurnLeft[j]))
			self.servo.setTarget(3,self.ServoComRight(self.Deg2BL_TurnLeft[j]))
			self.servo.setTarget(4,self.ServoComLeft(self.Deg1FR_TurnLeft[j]))
			self.servo.setTarget(5,self.ServoComRight(self.Deg2FR_TurnLeft[j]))

			#IsMoving()
			#time.sleep(0.005)
			timeDelay = max(0,t-time.time())
			print("timeDelay %.7f" %timeDelay)
			time.sleep(timeDelay)     #max is needed in Windows due to


	def Jump(self):
		leanOffset = -30.0
		Charge = -170.0
		jump = -310.0

		# charge 
		Ang0UpL, Ang1UpL = self.LeftLegINV(leanOffset, Charge)	
		Ang0UpR, Ang1UpR = self.RightLegINV(-leanOffset, Charge)

		# jump
		Ang0DownL, Ang1DownL = self.LeftLegINV(leanOffset, jump)
		Ang0DownR, Ang1DownR = self.RightLegINV(-leanOffset, jump)

		delayFlyTime = 1
		waitTime = 3

		## Go for charge
		# FL
		self.servo.setTarget(0,self.ServoComLeft(Ang0UpL))
		self.servo.setTarget(1,self.ServoComRight(Ang1UpL))
		# BL
		self.servo.setTarget(2,self.ServoComLeft(Ang0UpL))
		self.servo.setTarget(3,self.ServoComRight(Ang1UpL))
		# FR
		self.servo.setTarget(4,self.ServoComLeft(Ang0UpR))
		self.servo.setTarget(5,self.ServoComRight(Ang1UpR))
		# BR
		self.servo.setTarget(6,self.ServoComLeft(Ang0UpR))
		self.servo.setTarget(7,self.ServoComRight(Ang1UpR))

		time.sleep(waitTime)
		## Go for jump
		# FL
		self.servo.setTarget(0,self.ServoComLeft(Ang0DownL))
		self.servo.setTarget(1,self.ServoComRight(Ang1DownL))
		# BL
		self.servo.setTarget(2,self.ServoComLeft(Ang0DownL))
		self.servo.setTarget(3,self.ServoComRight(Ang1DownL))
		# FR
		self.servo.setTarget(4,self.ServoComLeft(Ang0DownR))
		self.servo.setTarget(5,self.ServoComRight(Ang1DownR))
		# BR
		self.servo.setTarget(6,self.ServoComLeft(Ang0DownR))
		self.servo.setTarget(7,self.ServoComRight(Ang1DownR))

		time.sleep(delayFlyTime)

		# FL
		self.servo.setTarget(0,self.ServoComLeft(Ang0UpL))
		self.servo.setTarget(1,self.ServoComRight(Ang1UpL))
		# BL
		self.servo.setTarget(2,self.ServoComLeft(Ang0UpL))
		self.servo.setTarget(3,self.ServoComRight(Ang1UpL))
		# FR
		self.servo.setTarget(4,self.ServoComLeft(Ang0UpR))
		self.servo.setTarget(5,self.ServoComRight(Ang1UpR))
		# BR
		self.servo.setTarget(6,self.ServoComLeft(Ang0UpR))
		self.servo.setTarget(7,self.ServoComRight(Ang1UpR))

		
	def Walk(self):
		T = self.stepWidth
		A = self.stepAmp
		yOffSet = self.stepHeight
		inc = 1

		########## Back Left ########## 

		# big step
		x1BL = np.arange(T/2,-T/2,-inc)
		y1BL = np.zeros(shape=np.shape(x1BL))
		for i in range(0,len(x1BL)):
			y1BL[i] = (A*m.sin((m.pi/T)*x1BL[i] + m.pi/2) + yOffSet)
		# drag
		x2BL = np.arange(-T/2,0,inc)
		y2BL = np.ones(len(x2BL))*yOffSet
		# stay
		x3BL = np.zeros(shape=np.shape(x1BL))
		y3BL = np.ones(len(x3BL))*yOffSet
		# drag
		x4BL = np.arange(0,T/2,inc)
		y4BL = np.ones(len(x4BL))*yOffSet

		xBL = np.concatenate((x1BL,x2BL,x3BL,x4BL))
		yBL = np.concatenate((y1BL,y2BL,y3BL,y4BL))

		########## Front Left ########## 
		
		# stay
		x1FL = np.zeros(shape=np.shape(x1BL))
		y1FL = np.ones(len(x1FL))*yOffSet
		# small step
		x2FL = np.arange(0,-T/2,-inc)
		y2FL = np.zeros(shape=np.shape(x2FL))
		for i in range(0,len(x2FL)):
			y2FL[i] = (A*m.sin(( (2*m.pi)/T )*x2FL[i] + m.pi) + yOffSet)
		# stay
		x3FL = np.ones(len(x1FL))*(-T/2)
		y3FL = np.ones(len(x3FL))*yOffSet
		# drag
		x4FL = np.arange(-T/2,0,inc)
		y4FL = np.ones(len(x4FL))*yOffSet

		xFL = np.concatenate((x1FL,x2FL,x3FL,x4FL))
		yFL = np.concatenate((y1FL,y2FL,y3FL,y4FL))

		########## Back Right ##########

		# stay
		x1BR = np.zeros(shape=np.shape(x1BL))
		y1BR = np.ones(len(x1BR))*yOffSet	 
		# drag
		x2BR = np.arange(0,-T/2,-inc)
		y2BR = np.ones(len(x2BR))*yOffSet
		# big step
		x3BR = np.arange(-T/2,T/2,inc)
		y3BR = np.zeros(shape=np.shape(x3BR))
		for i in range(0,len(x3BR)):
			y3BR[i] = (A*m.sin((m.pi/T)*x3BR[i] + m.pi/2) + yOffSet)
		# drag
		x4BR = np.arange(T/2,0,-inc)
		y4BR = np.ones(len(x4BR))*yOffSet

		xBR = np.concatenate((x1BR,x2BR,x3BR,x4BR))
		yBR = np.concatenate((y1BR,y2BR,y3BR,y4BR))

		########## Front Right ##########

		# stay
		x1FR = np.ones(len(x1BL))*(T/2)
		y1FR = np.ones(len(x1FR))*yOffSet	
		# drag
		x2FR = np.arange(T/2,0,-inc)
		y2FR = np.ones(len(x2FR))*yOffSet
		# stay
		x3FR = np.zeros(shape=np.shape(x3BR))
		y3FR = np.ones(len(x3FR))*yOffSet
		# small step
		x4FR = np.arange(0,T/2,inc)
		y4FR = np.zeros(shape=np.shape(x4FR))
		for i in range(0,len(x4FR)):
			y4FR[i] = (-A*m.sin(( (2*m.pi)/T )*x4FR[i] + m.pi) + yOffSet)

		xFR = np.concatenate((x1FR,x2FR,x3FR,x4FR))
		yFR = np.concatenate((y1FR,y2FR,y3FR,y4FR))

		#plt.figure(1)
		#plt.plot(xFR,yFR)
		#plt.show()
		#quit()

		### Calculate Servo Angle ###
		Ang1BL = list()
		Ang2BL = list()
		Ang1FL = list()
		Ang2FL = list()
		Ang1BR = list()
		Ang2BR = list()
		Ang1FR = list()
		Ang2FR = list()

		for i in range(0,len(xBL)):
			ang1BL, ang2BL = self.LegINV(xBL[i],yBL[i])
			ang1FL, ang2FL = self.LegINV(xFL[i],yFL[i])
			ang1BR, ang2BR = self.LegINV(xBR[i],yBR[i])
			ang1FR, ang2FR = self.LegINV(xFR[i],yFR[i])

			Ang1BL.append(ang1BL)
			Ang2BL.append(ang2BL)

			Ang1FL.append(ang1FL)
			Ang2FL.append(ang2FL)

			Ang1BR.append(ang1BR)
			Ang2BR.append(ang2BR)

			Ang1FR.append(ang1FR)
			Ang2FR.append(ang2FR)

		### Prepare to walk, start at first point ###
		## BL ##
		self.servo.setTarget(0,self.ServoComLeft(Ang1BL[0]))
		self.servo.setTarget(1,self.ServoComRight(Ang2BL[0]))
		## FL ##
		self.servo.setTarget(2,self.ServoComLeft(Ang1FL[0]))
		self.servo.setTarget(3,self.ServoComRight(Ang2FL[0]))
		## BR ##
		self.servo.setTarget(4,self.ServoComLeft(Ang1BR[0]))
		self.servo.setTarget(5,self.ServoComRight(Ang2BR[0]))
		## FR ##
		self.servo.setTarget(6,self.ServoComLeft(Ang1FR[0]))
		self.servo.setTarget(7,self.ServoComRight(Ang2FR[0]))

		time.sleep(2)
		### The robot walk as generated pattern ###
		samplingTime = 0.008     # the delay should not be lower than 0.005 sec
		T = time.time()
		while True:
			for i in range(0,len(Ang1BL)):
				T=T+samplingTime
				startTime = time.time()
				## BL ##
				self.servo.setTarget(2,self.ServoComLeft(Ang1BL[i]))
				self.servo.setTarget(3,self.ServoComRight(Ang2BL[i]))
				## FL ##
				self.servo.setTarget(0,self.ServoComLeft(Ang1FL[i]))
				self.servo.setTarget(1,self.ServoComRight(Ang2FL[i]))
				## BR ##
				self.servo.setTarget(6,self.ServoComLeft(Ang1BR[i]))
				self.servo.setTarget(7,self.ServoComRight(Ang2BR[i]))
				## FR ##
				self.servo.setTarget(4,self.ServoComLeft(Ang1FR[i]))
				self.servo.setTarget(5,self.ServoComRight(Ang2FR[i]))

				#IsMoving()

				time.sleep(max(0,T-time.time()))     #max is needed in Windows due to

		