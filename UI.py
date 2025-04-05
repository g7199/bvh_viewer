import imgui
import os
from tkinter import filedialog
from BVH_Parser import BVHParser

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
    imgui.end()


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
            state['animations'] = []
            file_paths = list(file_paths)  # 여러개의 애니메이션 파일을 다루기 때문에 list로 변환
            for path in file_paths:
                root, motion_frames = BVHParser(path)
                animation_data = {
                    'path': os.path.basename(path),
                    'root': root,
                    'motion_frames': motion_frames,
                    'frame_len': motion_frames.frameCount
                }
                state['animations'].append(animation_data)

    # 애니메이션 이름 표시
    imgui.text("Loaded Animations:")
    for i, data in enumerate(state['animations']):
        imgui.bullet_text(f"{i}: {data['path']}")

    imgui.end()
