# events.py
import math
from pygame.locals import *
import pygame
from pyglm import glm

def update_eye(center, distance, yaw, pitch):
    max_pitch = math.radians(89)
    pitch = max(-max_pitch, min(pitch, max_pitch))
    offset_x = distance * math.sin(yaw) * math.cos(pitch)
    offset_y = distance * math.sin(pitch)
    offset_z = distance * math.cos(yaw) * math.cos(pitch)
    return center + glm.vec3(offset_x, offset_y, offset_z)

def handle_mouse_motion(event, state):
    # 'state' is a dictionary or an object containing last_x, last_y, yaw, pitch, center, eye, distance, etc.
    xpos, ypos = event.pos
    dx = xpos - state['last_x']
    dy = ypos - state['last_y']
    state['last_x'], state['last_y'] = xpos, ypos

    if state['is_rotating']:
        sensitivity = 0.005
        state['yaw'] -= dx * sensitivity
        state['pitch'] += dy * sensitivity
        state['eye'] = update_eye(state['center'], state['distance'], state['yaw'], state['pitch'])
    elif state['is_translating']:
        sensitivity = 0.005 * state['distance']
        view_dir = glm.normalize(state['center'] - state['eye'])
        right = glm.normalize(glm.cross(view_dir, glm.vec3(0, 1, 0)))
        up = glm.vec3(0, 1, 0)
        translation = (-dx * right + dy * up) * sensitivity
        state['center'] += translation
        state['eye'] = update_eye(state['center'], state['distance'], state['yaw'], state['pitch'])

def handle_mouse_button(event, state):
    if event.type == pygame.MOUSEBUTTONDOWN:
        state['last_x'], state['last_y'] = event.pos
        if event.button == 1:
            state['is_rotating'] = True
        elif event.button == 3:
            state['is_translating'] = True
        if event.button in (4, 5):
            handle_mouse_wheel(event, state)
    elif event.type == pygame.MOUSEBUTTONUP:
        if event.button == 1:
            state['is_rotating'] = False
        elif event.button == 3:
            state['is_translating'] = False

def handle_mouse_wheel(event, state):
    zoom_sensitivity = 0.1
    if hasattr(event, 'y'):
        state['distance'] -= zoom_sensitivity * event.y * state['distance']
    else:
        if event.button == 4:
            state['distance'] -= zoom_sensitivity * state['distance']
        elif event.button == 5:
            state['distance'] += zoom_sensitivity * state['distance']
    state['distance'] = max(state['distance'], 0.1)
    state['eye'] = update_eye(state['center'], state['distance'], state['yaw'], state['pitch'])
