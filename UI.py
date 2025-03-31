import imgui
import os
from tkinter import filedialog
from BVH_Parser import bvh_parser

def draw_control_panel(state):
    """
    Animation의 재생/멈춤 그리고 slider 바를 표시하는 Control Panel입니다.
    :param state: Animation 정보를 담고있는 딕셔너리
    """
    imgui.set_next_window_position(10, 10)
    imgui.begin("Control Panel")
    changed, value = imgui.slider_int("Slider", state['frame_idx']+1, 0, state['frame_len'])
    if imgui.button("play/pause", 100, 30):
        state['stop'] = not state['stop']
    if changed:
        state['frame_idx'] = value-1
    imgui.separator()
    imgui.text("Joint Tree:")
    if state['root']:
        draw_joint_tree(state['root'])
    imgui.end()

def draw_joint_tree(joint):
    """
    Skeleton의 Hierarchy를 표시하는 창입니다.

    :param joint: 계층별로 존재하는 Joint를 그리기 위함입니다.
    """
    node_open = imgui.tree_node(joint.name)
    if imgui.is_item_clicked():
        print("Joint selected:", joint.name)
    if node_open:
        for child in joint.children:
            draw_joint_tree(child)
        imgui.tree_pop()

def draw_file_loader(state):
    """
    BVH_Data를 폴더 탐색기에서 직접 I/O 하기 위한 창입니다.
    :param state: 현재 선택된 파일 경로를 담고 있는 state 딕셔너리
    """
    imgui.set_next_window_position(550, 10, condition=imgui.ONCE)
    imgui.set_next_window_size(300, 200, condition=imgui.ONCE)
    imgui.begin("BVH Loader")

    # Load 버튼
    if imgui.button("Open BVH Files"):
        file_paths = filedialog.askopenfilenames(
            title="Select BVH files",
            filetypes=[("BVH Files", "*.bvh")]
        )
        if file_paths:
            file_paths = list(file_paths)  # 여러개의 애니메이션 파일을 다루기 때문에 list로 변환
            state['loaded_file_paths'] = file_paths
            state['animations'] = []
            for path in file_paths:
                root, motion_frames = bvh_parser(path)
                animation_data = {
                    'root': root,
                    'motion_frames': motion_frames,
                    'frame_len': len(motion_frames)
                }
                state['animations'].append(animation_data)
            state['current_animation'] = 0  # 다시 load시 첫번쨰 animation에서 시작
            state['blend'] = 0.8  # 기본 blend factor
            state['root'], state['motion_frames'] = state['animations'][0]['root'],state['animations'][0]['motion_frames']
            state['frame_len'] = state['animations'][0]['frame_len']
            state['frame_idx'] = 0

    # 애니메이션 이름 표시
    if 'loaded_file_paths' in state and state['loaded_file_paths']:
        imgui.text("Loaded Animations:")
        for i, path in enumerate(state['loaded_file_paths']):
            imgui.bullet_text(f"{i}: {os.path.basename(path)}")

        # Blend slider: Blend될 위치
        changed_blend, blend = imgui.slider_float("Blend Factor", state.get('blend', 0.8), 0.0, 1.0)
        if changed_blend:
            state['blend'] = blend

    imgui.end()
