import pygame
import time

pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x)
             for x in range(pygame.joystick.get_count())]

if len(joysticks) == 0:
    print("No joysticks found")

joystick = joysticks[0]

while True:
    time.sleep(0.5)
    axes = joystick.get_numaxes()
    for i in range(axes):
        axis = joystick.get_axis(i)
        print(axis)
