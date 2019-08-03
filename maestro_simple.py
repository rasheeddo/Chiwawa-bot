# Test servo with Maestro 

import maestro
import time

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

servo = maestro.Controller()
#acc = 10
#servo.setAccel(0,acc)      #set servo 0 acceleration to 4
#servo.setAccel(1,acc)      #set servo 0 acceleration to 4
#servo.setAccel(2,acc)      #set servo 0 acceleration to 4

#speed = 10
#servo.setSpeed(0,speed)
#servo.setSpeed(1,speed)
#servo.setSpeed(2,speed)

#pos = 40
#servo_pos = map(pos, 0, 90, 4000, 8000)
while True:

	servo.setTarget(0,4000)  #set servo to move to center position  min=3000  mid=6000  max=9000
	time.sleep(2)
	servo.setTarget(0,8000)  #set servo to move to center position  min=3000  mid=6000  max=9000
	time.sleep(2)


#print("input_pos=%f" %pos)
#print("servo_pos=%f" %servo_pos)
print("-----------------------------")


servo.close