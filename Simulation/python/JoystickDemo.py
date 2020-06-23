#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May  6 09:52:04 2020

@author: rega0051
"""

import pygame

pygame.init()

# Set up the joystick
pygame.joystick.init()

# Enumerate joysticks
joyList = []
for i in range(0, pygame.joystick.get_count()):
    joyList.append(pygame.joystick.Joystick(i).get_name())
    
print(joyList)


# By default, load the first available joystick.
if (len(joyList) > 0):
    joy = pygame.joystick.Joystick(0)
    joy.init()


print('Joystick: {}'.format(joy.get_name()))
print('  Axes: {}'.format(joy.get_numaxes()))
print('  Buttons: {}'.format(joy.get_numbuttons()))
print('  Hats: {}'.format(joy.get_numhats()))
print('  Balls: {}'.format(joy.get_numballs()))
print()


while True:
    pygame.event.get(pump=True) # Pump, retreive events so they clear
    
    # Read all the joystick values
    axes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
    buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
    hats = [joy.get_hat(i) for i in range(joy.get_numhats())]
    balls = [joy.get_ball(i) for i in range(joy.get_numballs())]
    
    print('Axis: ', axes, 'Button: ', buttons)
    
    if axes[-1] > 0.75:
        break

pygame.quit()
