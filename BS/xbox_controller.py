import pygame
import time

pygame.init()
pygame.joystick.init() # Explicitly initialize the joystick module

joysticks = pygame.joystick.get_count()
print(f"Number of joysticks detected: {joysticks}")

if joysticks > 0:
    # Get instance of the first joystick
    js = pygame.joystick.Joystick(0)
    print(f"Initialized Joystick: {js.get_name()}")