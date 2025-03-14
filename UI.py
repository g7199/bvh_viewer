# ui.py
import imgui
from tkinter import filedialog
from BVH_Parser import bvh_parser
def draw_control_panel(state):
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
    node_open = imgui.tree_node(joint.name)
    if imgui.is_item_clicked():
        print("Joint selected:", joint.name)
    if node_open:
        for child in joint.children:
            draw_joint_tree(child)
        imgui.tree_pop()

def draw_file_loader(state):
    imgui.set_next_window_position(550, 10, condition=imgui.ONCE)
    imgui.set_next_window_size(200, 100, condition=imgui.ONCE)
    imgui.begin("BVH Loader")
    if imgui.button("Open Folder/Directory"):
        file_path = filedialog.askopenfilename(
            title="Select BVH file",
            filetypes=[("BVH Files", "*.bvh")]
        )
        if file_path:
            state['loaded_file_path'] = file_path
            root, motion_frames = bvh_parser(file_path)
            state['root'] = root
            state['motion_frames'] = motion_frames
            print("File selected:", file_path)
    if state.get('loaded_file_path'):
        imgui.text("Loaded: {}".format(state['loaded_file_path'].split("/")[-1]))
    imgui.end()
