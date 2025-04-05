import argparse
import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import imgui
from imgui.integrations.pygame import PygameRenderer
from pyglm import glm
from BVH_Parser import BVHParser
from Rendering import draw_humanoid, draw_virtual_root_axis
from utils import draw_axes, set_lights
import Events
import UI
import os
state = {
        'center': glm.vec3(0, 0, 0),
        'eye': glm.vec3(60, 180, 600),
        'upVector': glm.vec3(0, 1, 0),
        'distance': glm.length(glm.vec3(60, 180, 600) - glm.vec3(0, 0, 0)),
        'yaw': math.atan2(60, 600),
        'pitch': math.asin((180) / glm.length(glm.vec3(60, 180, 600))),
        'last_x': 0,
        'last_y': 0,
        'is_rotating': False,
        'is_translating': False,
        'stop': False,
        'frame_idx': 0,
        'frame_len': 0,
        'animations': [],
    }

def resize(width, height):
    """
    glViewport사이즈를 조절하는 함수
    :param width: 너비
    :param height: 높이
    """

    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, width / height, 0.1, 5000.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def apply_virtual_root_offset(current_motion, next_motion):
    # Get the final frame of the current animation
    last_frame = current_motion.frameData[-1]
    prev_vr_pos = glm.vec3(*last_frame.position["VirtualRoot"])
    prev_vr_rot = last_frame.rotation["VirtualRoot"]

    # Get the first frame of the next animation
    first_frame = next_motion.frameData[0]
    next_vr_pos = glm.vec3(*first_frame.position["VirtualRoot"])
    next_vr_rot = first_frame.rotation["VirtualRoot"]

    # Compute the offset transform
    position_offset = prev_vr_pos - next_vr_pos
    rotation_offset = prev_vr_rot * glm.inverse(next_vr_rot)

    # Apply the offset to every frame of the next animation
    for frame in next_motion.frameData:
        # Update hip position by applying the position offset
        hip_vec = glm.vec3(*frame.position["hip"])
        hip_vec += position_offset
        frame.position["hip"] = [hip_vec.x, hip_vec.y, hip_vec.z]

        # Update virtual root position similarly
        vr_vec = glm.vec3(*frame.position["VirtualRoot"])
        vr_vec += position_offset
        frame.position["VirtualRoot"] = [vr_vec.x, vr_vec.y, vr_vec.z]

        # Update rotations by applying the rotation offset
        frame.rotation["hip"] = rotation_offset * frame.rotation["hip"]
        frame.rotation["VirtualRoot"] = rotation_offset * frame.rotation["VirtualRoot"]

def main():
    """
    BVH_Viewer 의 main loop
    """
    currentAnimIndex = 0
    pygame.init()
    size = (800, 600)
    screen = pygame.display.set_mode(size, pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)
    pygame.display.set_caption("BVH Viewer")
    glEnable(GL_DEPTH_TEST)
    set_lights()
    resize(*size)

    imgui.create_context()
    impl = PygameRenderer()

    clock = pygame.time.Clock()
    previous_time = pygame.time.get_ticks() / 1000.0
    frame_duration = 1 / 60.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                continue
            impl.process_event(event)
            io = imgui.get_io()

            if event.type == pygame.MOUSEWHEEL:
                if not io.want_capture_mouse:
                    Events.handle_mouse_wheel(event, state)
            if event.type == pygame.MOUSEMOTION:
                if not io.want_capture_mouse:
                    Events.handle_mouse_motion(event, state)
            if event.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP):
                if not io.want_capture_mouse:
                    Events.handle_mouse_button(event, state)
            if event.type == pygame.VIDEORESIZE:
                size = event.size
                screen = pygame.display.set_mode(size, pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)
                resize(*size)

        io.display_size = pygame.display.get_surface().get_size()
        current_time = pygame.time.get_ticks() / 1000.0
        delta_time = current_time - previous_time
        if not state['stop']:
            if delta_time >= frame_duration:
                state['frame_idx'] += 1
                if(state['frame_idx'] >= state['frame_len']):
                    currentAnimIndex = (currentAnimIndex+1)%len(state['animations'])
                    state['frame_len'] = state['animations'][currentAnimIndex]['frame_len']
                    state['frame_idx'] = 0
                previous_time = current_time

        imgui.new_frame()
        UI.draw_control_panel(state)
        UI.draw_file_loader(state)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluLookAt(state['eye'].x, state['eye'].y, state['eye'].z,
                  state['center'].x, state['center'].y, state['center'].z,
                  state['upVector'].x, state['upVector'].y, state['upVector'].z)
        draw_axes()


        draw_humanoid(state['animations'][currentAnimIndex]['root'],state['animations'][currentAnimIndex]['motion'],state['frame_idx'])
        draw_virtual_root_axis(state['animations'][currentAnimIndex]['root'],20)

        imgui.render()
        impl.render(imgui.get_draw_data())
        pygame.display.flip()
        clock.tick(60)

    impl.shutdown()
    pygame.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path")
    args = parser.parse_args()


    root, motion = BVHParser(args.file_path)
    state['frame_len'] = motion.frameCount

    animation_data = {
        'path': os.path.basename(args.file_path),
        'root': root,
        'motion': motion,
        'frame_len': motion.frameCount,
        'index': 0
    }
    state['animations'].append(animation_data)
    main()
