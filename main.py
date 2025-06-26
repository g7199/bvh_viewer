#main.py
import argparse
import math
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
import imgui
from imgui.integrations.pygame import PygameRenderer
from pyglm import glm
#import numpy as np

from BVH_Parser import bvh_parser, check_bvh_structure, motion_connect
from Rendering import draw_humanoid, draw_virtual_root_axis
from utils import draw_axes, set_lights, random_color
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
        #'frame_idx': 0,
        #'frame_len': None,
        'root': None,
        'motion_frames': [],
        'motion' : None,
        'loaded_file_path': None,

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


def main():
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
                state['frame_idx'] = (state['frame_idx'] + 1) % state['frame_len']
                previous_time = current_time

                #import time
                #time.sleep(1)

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
            frame = state['motion_frames'][state['frame_idx']]
            if frame.virtual_transform is None:
                print(f"Frame {state['frame_idx']}: virtual_transform is None!")
            else:
                # 행렬의 4번째 열이 위치 정보입니다.
                pos_from_matrix = glm.vec3(frame.virtual_transform[3])
                if glm.length(pos_from_matrix) < 1e-4 and state['frame_idx'] > 10:  # 처음 몇 프레임 제외
                    # 프레임 인덱스와 위치 정보를 출력
                    print(
                        f"Frame {state['frame_idx']}: Low translation in virtual_transform -> {pos_from_matrix.x:.2f}, {pos_from_matrix.y:.2f}, {pos_from_matrix.z:.2f}")
            glPushMatrix()
            draw_humanoid(state['root'], frame, random_color())

            #hip_node = state['root'].children[0]
            virtual_root_t = frame.virtual_transform
            if virtual_root_t is None:
                print("🚨 virtual_root_T is None! Can't draw axis.")
            else:
                draw_virtual_root_axis(virtual_root_t, random_color())
            glPopMatrix()

        imgui.render()
        impl.render(imgui.get_draw_data())
        pygame.display.flip()
        clock.tick(60)

    impl.shutdown()
    pygame.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_paths", nargs=2)
    args = parser.parse_args()

    root1, motion1 = bvh_parser(args.file_paths[0])
    root2, motion2 = bvh_parser(args.file_paths[1])

    check_bvh_structure(root1, is_root=True)
    check_bvh_structure(root2, is_root=True)

    # 모션 연결 (파라미터는 원하는 대로 조절)
    connected_motion = motion_connect(motion1, motion2, root1, transition_frames=60)

    # 이제 이 모션을 그려야 하므로, state에 등록
    state['root'] = root1  # <- 어떤 골격으로 그릴지
    state['motion'] = connected_motion
    state['motion_frames'] = connected_motion.quaternion_frame  # <- 연결된 모션
    # frame 개수 저장
    state['frame_len'] = len(connected_motion)
    state['frame_idx'] = 0  # 초기화

    main()
