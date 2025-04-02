import argparse
import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import imgui
from imgui.integrations.pygame import PygameRenderer
from pyglm import glm

from BVH_Parser import bvh_parser, check_bvh_structure
from Transforms import motion_adapter, extract_yaw_rotation, interpolate_frames, add_motion, inverse_matrix, interpolate_frames_with_quat
from Rendering import draw_humanoid, draw_virtual_root_axis
from utils import draw_axes, set_lights
import Events
import UI

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
        'frame_len': None,
        'root': None,
        'motion_frames': None,
        'loaded_file_path': None,
        'current_animation': 0,
        'animations': [],
        'isInterpolating': False,
        'blend': 0.8,
        'T_offset': [0,0,0],
    }
local_frame = 0
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


def main():
    global local_frame
    """
    BVH_Viewer 의 main loop
    """
    pygame.init()
    size = (800, 600)
    screen = pygame.display.set_mode(size, pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)
    pygame.display.set_caption("BVH Viewer with ImGui Control Panel")
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
            if delta_time >= frame_duration and state['motion_frames']:
                state['frame_idx'] += 1
                if(state['frame_idx'] >= state['frame_len']):
                    state['frame_idx'] = 0
                    state['current_animation'] = (state['current_animation'] + 1) % len(state['animations'])
                    state['root'] = state['animations'][state['current_animation']]['root']
                    state['motion_frames'] = state['animations'][state['current_animation']]['motion_frames']
                    state['frame_len'] = state['animations'][state['current_animation']]['frame_len']

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
        if state['motion_frames'] and state['root']:
            progress = state['frame_idx'] / state['frame_len']
            next_idx = (state['current_animation'] + 1) % len(state['animations'])
            next_animation = state['animations'][next_idx]
            next_motion_frames = next_animation['motion_frames']
            next_root = next_animation['root']

            if state['blend'] <= progress:
                state['isInterpolating'] = True
                t = (progress - state['blend']) / (1-state['blend'])

                current_frame = state['motion_frames'][state['frame_idx']]
                next_frame = next_motion_frames[local_frame % len(next_motion_frames)]
                next_frame[:3] = current_frame[:3]
                blended_frame = interpolate_frames_with_quat(current_frame, next_frame, t, state['root'])
                root_position, _ = motion_adapter(state['root'], blended_frame)
                #root_position, _ = motion_adapter(state['root'], state['motion_frames'][state['frame_idx']])
                local_frame += 1 
            else:
                if(state['isInterpolating'] == True):
                    state['frame_idx'] = local_frame
                    state['isInterpolating'] = False
                local_frame = 0
                root_position, _ = motion_adapter(state['root'], state['motion_frames'][state['frame_idx']])


            draw_humanoid(root_position, state['root'])
            hip_node = state['root'].children[0]
            hip_rotation = extract_yaw_rotation(hip_node.kinetics)
            draw_virtual_root_axis(state['root'], hip_rotation, axis_length=30.0)

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


    root, motion_frames = bvh_parser(args.file_path)
    state['frame_len'] = len(motion_frames)
    check_bvh_structure(root, is_root=True)

    state['root'] = root
    state['motion_frames'] = motion_frames
    animation_data = {
        'root': root,
        'motion_frames': motion_frames,
        'frame_len': len(motion_frames)
    }
    state['animations'].append(animation_data)

    main()
