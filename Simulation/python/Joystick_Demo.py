#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May  6 09:52:04 2020

@author: rega0051
"""

import sdl2

class Joystick:
    def __init__(self, joyName = b'FrSky Taranis Joystick'):
        # Set up the joystick
        sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)

        # Enumerate joysticks
        joyList = []
        for i in range(0, sdl2.SDL_NumJoysticks()):
            joyList.append(sdl2.SDL_JoystickNameForIndex(i))

        print(joyList)

        # By default, load the first available joystick or joyName.
        if (len(joyList) > 0):
            indxJoy = 0
            try:
                indxJoy = joyList.index(joyName)
            except:
                pass

            self.joy = sdl2.SDL_JoystickOpen(indxJoy)
        else:
            print("Joystick Fail!")

        print( "Joystick Name:",          sdl2.SDL_JoystickName(self.joy))
        print( "Joystick NumAxes",        sdl2.SDL_JoystickNumAxes(self.joy))
        # print( "Joystick NumTrackballs:",    sdl2.SDL_JoystickNumBalls(self.joy))
        print( "Joystick Buttons:",       sdl2.SDL_JoystickNumButtons(self.joy))
        # print( "Joystick NumHats:",          sdl2.SDL_JoystickNumHats(self.joy))

        self.axis = []
        self.button = []
        self.msg = [0.0] * 16

    def update(self):
        # sdl2.SDL_PumpEvents() # Pump, retreive events so they clear
        sdl2.SDL_JoystickUpdate()

        # Read all the joystick values
        axisScale = 1 / 2**15
        self.axis = [sdl2.SDL_JoystickGetAxis(self.joy, i) for i in range(sdl2.SDL_JoystickNumAxes(self.joy))]
        self.button = [sdl2.SDL_JoystickGetButton(self.joy, i) for i in range(sdl2.SDL_JoystickNumButtons(self.joy))]

        self.roll =  axisScale * self.axis[0]
        self.pitch =  axisScale * self.axis[1]
        self.throttle =  axisScale * self.axis[2]
        self.yaw =  axisScale * self.axis[3]

        self.flap = 0.0

        self.autoEngage = 2 * self.button[0] - 1 # "Soc-Engage-Switch"
        self.testMode = axisScale * self.axis[4] # "Test-Mode-Switch"
        self.testSel = axisScale * self.axis[5] # "Test-Select-Switch"
        self.trig = 2*self.button[1]-1 # "Trigger-Switch"
        self.thrSafe = 2 * self.button[2] - 1 # "Throttle-Safety-Switch"

        self.baseSel = self.axis[7] # "Baseline-Select-Switch"

    def sbus(self):

        self.msg[0] = self.autoEngage
        self.msg[1] = self.thrSafe
        self.msg[2] = 0
        self.msg[3] = self.roll
        self.msg[4] = self.pitch
        self.msg[5] = self.yaw
        self.msg[6] = self.flap
        self.msg[7] = self.throttle
        self.msg[8] = self.testMode
        self.msg[9] = self.testSel
        self.msg[10] = self.trig
        self.msg[11] = self.baseSel

        return self.msg

    def __del__(self):
        sdl2.SDL_Quit()


# Joystick Map, FIXIT - this is hacky
# OpenTX Mixer: 1-Roll, 2-Pitch, 3-Thrt, 4-Yaw, 5-SA, 6-SB, 7-SC, 8-<blank>
# SBUS def from thor.json:

joystick = Joystick()

# Run the joustick just to populate messages
joystick.update()
joystick.sbus()

#%%
import time

while True:
    # sdl2.SDL_PumpEvents() # Pump, retreive events so they clear
    joystick.update()
    joystick.sbus()

    print('Axis: ', joystick.axis)
    print('Button: ', joystick.button[0:3])
    print('MSG: ', joystick.msg)

    if joystick.axis[-1] > 0.75:
        break

    time.sleep(0.1)
