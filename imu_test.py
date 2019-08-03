#Script for testing filtering on the MPU data:

from mpu6050 import mpu6050
import math 
import numpy as np
import time

sensor = mpu6050(0x68)

def getIMUAngle():
        global init,Kpitch,Kroll,Kyaw
        accelerometer_data = sensor.get_accel_data()
        giroscope_data=sensor.get_gyro_data()
        
        rawX=accelerometer_data["x"]
        rawY=accelerometer_data["y"]
        rawZ=accelerometer_data["z"]
        rawXp=giroscope_data["x"]
        rawYp=-giroscope_data["y"]
        rawZp=giroscope_data["z"]
        
        #Angle calculation by axelerometer
        pitchA=m.degrees((m.atan2(rawX,m.sqrt(m.pow(rawY,2)+m.pow(rawZ,2)))))
        rollA=m.degrees((m.atan2(rawY,m.sqrt(m.pow(rawX,2)+m.pow(rawZ,2)))))
        yawA=m.degrees((m.atan2(m.sqrt(m.pow(rawX,2)+m.pow(rawY,2)),rawZ)))

        #Get angle variation from gyroscope:
        if(init):
            pitchG=pitchA
            rollG=rollA
            yawG=0
            init=0
            print("init")
        else:
            #Update the position
            pitchG=Dt*rawYp+Kpitch
            rollG=Dt*rawXp+Kroll
            yawG=Dt*rawZp+Kyaw

        #Fusion with Kalman filter (not tunned)
        Kpitch=k_p*pitchG+(1-k_p)*pitchA
        Kroll=k_r*rollG+(1-k_r)*rollA
        Kyaw=k*yawG+(1-k)*yawA

        return Kpitch,Kroll,Kyaw

def getIMU3():
    global init3, pitch_out, roll_out
    Dt=0.02
    acc_data = sensor.get_accel_data()
    gyro_data=sensor.get_gyro_data()

    accX=acc_data["x"]
    accY=acc_data["y"]
    accZ=acc_data["z"]
    gyroX=gyro_data["x"] - GyroXcal
    gyroY=gyro_data["y"] - GyroYcal
    gyroZ=gyro_data["z"] - GyroZcal


    #Angle calculation by axelerometer
    roll_acc = math.degrees(math.atan(accY/math.sqrt(math.pow(accX,2)+math.pow(accZ,2))))
    pitch_acc = math.degrees(math.atan(-accX/math.sqrt(math.pow(accY,2)+math.pow(accZ,2))))
    #yawA=m.degrees((m.atan(m.sqrt(m.pow(accX,2)+m.pow(accY,2)),rawZ)))
    
    if (init3):
        roll_gyro = roll_acc
        pitch_gyro = pitch_acc
        init3 = 0
    else:
        roll_gyro = Dt*gyroX + roll_out
        pitch_gyro = Dt*gyroY + pitch_out
    

    roll_out = kr*roll_gyro + (1-kr)*roll_acc
    pitch_out = kp*pitch_gyro + (1-kp)*pitch_acc

    return roll_out, pitch_out

def calibrationOffset():
    print("Start calibration....")
    n=100
    X=0
    Y=0
    Z=0
    for a in range(0,n):
        V=getIMUAngle()
        X=V[0]+X
        Y=V[1]+Y
        Z=V[2]+Z
    X_offset=-X/n
    Y_offset=-Y/n
    Z_offset=-Z/n
    time.sleep(0.04)
    print("....Finished")
    print("offsets:",X_offset,Y_offset,Z_offset)
    return X_offset,Y_offset,Z_offset

def smoother(n,rawPitch,rawRoll,rawYaw):
        #Do the average on the 5 last measured value to smooth the sensor output
        global initAcq
        if(initAcq):
                initAcq=0
                for i in range(0,n):
                        avPitch.append(rawPitch)
                        avRoll.append(rawRoll)
                        avYaw.append(rawYaw)
                        
        else:
                avPitch.pop(0)
                avPitch.append(rawPitch)
                avRoll.pop(0)
                avRoll.append(rawRoll)
                avYaw.pop(0)
                avYaw.append(rawYaw)
                
        return int(sum(avPitch)/len(avPitch)),int(sum(avRoll)/len(avRoll)),int(sum(avYaw)/len(avYaw))

def getRawGyro():
    giroscope_data=sensor.get_gyro_data()
    rawXp=giroscope_data["x"]
    rawYp=giroscope_data["y"]
    rawZp=giroscope_data["z"]
    return rawXp, rawYp, rawZp

def RawGyroCalibration():
    print("Doing gyro calibration...(Keep stable!)")
    gyroX = []
    gyroY = []
    gyroZ = [] 
    for i in range(0,500):
        rawGyro = getRawGyro()
        gyroX.append(rawGyro[0])
        gyroY.append(rawGyro[1])
        gyroZ.append(rawGyro[2])
        time.sleep(0.01)

    ave_gyroX = sum(gyroX)/len(gyroX)
    ave_gyroY = sum(gyroY)/len(gyroY)
    ave_gyroZ = sum(gyroZ)/len(gyroZ)

    return ave_gyroX, ave_gyroY, ave_gyroZ

                   
pitch = []
roll = []
pitch2 = []
roll2 = []
yaw = []
smPitch = []
smRoll = []
smYaw = []

rawXp = []
rawYaw = []
rawYp = []
print("Get signal...")

gyroCal = RawGyroCalibration()
global GyroXcal, GyroYcal, GyroZcal
GyroXcal = gyroCal[0]
GyroYcal = gyroCal[1]
GyroZcal = gyroCal[2]


### Rasheed's method
init3 = 1
Dt = 0.02
global kr, kp
kr = 0.99
kp = 0.99
roll3= []
pitch3=[]

print("Start shaking!")
acqui=500
for n in range(0,acqui):


        full=getIMU3()
        roll2.append(full[0])
        pitch2.append(full[1])
        #yaw.append(full[2]+Z_offset)
        
        #smfull=smoother(3,pitch2[n],roll2[n],yaw[n])
        
        #smPitch.append(smfull[0])
        #smRoll.append(smfull[1])
        #smYaw.append(smfull[2])


        #giroscope_data=sensor.get_gyro_data()
        #rawXp.append(giroscope_data["x"])
        #rawYp.append(giroscope_data["y"])

        
        time.sleep(Dt)
                   
print("...Finished")

print("DATA visualisation:")
import matplotlib.pyplot as plt

t=np.arange(0,acqui,1)

plt.figure(1)

plt.subplot(2, 1, 1)
#plt.plot(t,pitch,'r')
plt.plot(t,pitch2,'b')
plt.grid()
plt.title("Pitch")

plt.subplot(2, 1, 2)
#plt.plot(t,roll,'r')
plt.plot(t,roll2,'b')
plt.grid()
plt.title("Roll")

'''
plt.subplot(3,1,3)
plt.plot(t,rawYp,'r',t,rawXp,'b')
plt.grid()
'''

plt.legend()
plt.show()





