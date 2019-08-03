from ChiwawaClass import *
import pygame

pygame.init()
j = pygame.joystick.Joystick(0)
j.init()

def getButton():
    #Read input from the two joysticks
    pygame.event.pump()

    button0 = j.get_button(0)
    button1 = j.get_button(1)
    button2 = j.get_button(2)
    button3 = j.get_button(3)
    button4 = j.get_button(4)
    button5 = j.get_button(5)
    button6 = j.get_button(6)
    button7 = j.get_button(7)
    button8 = j.get_button(8)
    button9 = j.get_button(9)
    button10 = j.get_button(10)
    button11 = j.get_button(11)
    button12 = j.get_button(12)

    joy_button = [button0, button1, button2, button3, button4, button5, button6, button7,button8, button9, button10, button11,button12]
    
    return joy_button

def getAxis():
    #Read input from the two joysticks
    pygame.event.pump()
    axis0 = j.get_axis(0)
    axis1 = j.get_axis(1)
    axis2 = j.get_axis(2)
    axis3 = j.get_axis(3)
    #axis4 = j.get_axis(4)
    #axis5 = j.get_axis(5)
    joy_axis = [axis0, axis1, axis2, axis3]
    return joy_axis

ch = Chiwawa()


ch.StandBy()

time.sleep(1)

#ch.Jump()
#quit()

#ch.Step(True)


while True:

    Buttons = getButton()
    B0 = Buttons[0] #X
    B1 = Buttons[1] #Y
    B2 = Buttons[2] #A
    B3 = Buttons[3] #B
    #B4 = Buttons[4] #LB
    #B5 = Buttons[5] #RB
    #B6 = Buttons[6] #LT
    #B7 = Buttons[7] #RT
    #B8 = Buttons[8] # Analog Left Push
    #B9 = Buttons[9] # Analog Right Push
    B10 = Buttons[10] # Back
    B11 = Buttons[11] # Start
    #B12 = Buttons[12] # Guide

    Axes = getAxis()
    Ax0 = Axes[0]       # Analog left push right = +1,  Analog left push left = -1
    Ax1 = Axes[1]       # Analog left push down = +1,  Analog left push up = -1
    Ax2 = Axes[2]       # Analog right push down = +1, Analog right push up = -1
    Ax3 = Axes[3]       # Analog right push right = +1, Analog right push left = -1

    gain = Ax0*50

    ch.PitchBody(gain)
    print()









#ch.Trot(True)

#ch.TurnRight(True)

#ch.TurnLeft(True)
#ch.Lean()

#ch.Step()

#ch.Jump()
