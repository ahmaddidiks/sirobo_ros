import sys
import pygame
from pygame.locals import *

pygame.init()

pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
for joystick in joysticks:
    joystick.init()
    print(f"Connected: {joystick.get_name()}")

motion = [0, 0]

while True:
    for event in pygame.event.get():
        if event.type == JOYBUTTONDOWN:
            print(f"Button pressed: {event.button}")
        if event.type == JOYBUTTONUP:
            print(f"Button released: {event.button}")
        if event.type == JOYAXISMOTION:
            print(f"Axis motion: axis {event.axis} value {event.value}")
            if event.axis < 2:
                motion[event.axis] = event.value
        if event.type == JOYHATMOTION:
            print(f"Hat motion: {event.value}")
        if event.type == JOYDEVICEADDED:
            joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
            for joystick in joysticks:
                print(f"Connected: {joystick.get_name()}")
        if event.type == JOYDEVICEREMOVED:
            joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
            print("Joystick disconnected")
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                pygame.quit()
                sys.exit()

    # Small delay to prevent spamming the terminal
    pygame.time.wait(10)
