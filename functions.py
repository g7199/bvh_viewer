import numpy as np

class Joint:
    def __init__(self, name):
        self.name = name
        self.channels = []
        self.children = []
        self.offset = [0,0,0]
        self.position = None
        self.rotation = []
        self.kinetics = np.identity(4, dtype=float)

def bvh_parser(file_path):
    stack = []
    root = None
    cur_node = None
    motion = []
    is_motion = False

    try:
        with open(file_path, "r", encoding="utf-8") as file:
            for line in file:
                parts = line.split()
                if not parts:
                    continue

                if is_motion:
                    if parts[0] in ["Frames:", "Frame"]:
                        continue
                    motion_frame = []
                    parts = line.split()

                    for part in parts:
                        motion_frame.append(part)
                    motion.append(motion_frame)
                    continue

                if parts[0] == "MOTION":
                    is_motion = True

                if parts[0] in ["ROOT", "JOINT", "End"]:
                    node = Joint(parts[1])
                    node.parent = cur_node
                    if not root:
                        root = node
                    if cur_node:
                        cur_node.children.append(node)

                    stack.append(node)
                    cur_node = node

                elif parts[0] == "OFFSET":
                    cur_node.offset = list(map(float, parts[1:]))

                elif parts[0] == "CHANNELS":
                    cur_node.channels = parts[2:]

                elif parts[0] == "}":
                    stack.pop()
                    if stack:
                        cur_node = stack[-1]

        return root, motion

    except FileNotFoundError:
        print(f"Error: File '{file_path}' does not exist.")

def print_bvh_tree(node, depth=0):
    if not node:
        return
    indent = "  " * depth

    if node.channels:
        print(f"{indent}{node.name} Children:{node.children}")
    else:
        print(f"{indent}{node.name} Children:{node.children}")

    for child in node.children:
        print_bvh_tree(child, depth + 1)

def add_motion(node, motion_frame, idx=[0]):
    if not node:
        return

    if node.name != "Site":
        if len(node.channels) == 6:
            node.position = list(map(float, motion_frame[idx[0]:idx[0]+3]))
            idx[0] += 3
            node.rotation = list(map(float, motion_frame[idx[0]:idx[0]+3]))
            node.kinetics = compute_forward_kinetics(node, node.rotation)
            idx[0] += 3
        elif len(node.channels) == 3:
            node.rotation = list(map(float, motion_frame[idx[0]:idx[0]+3]))
            node.kinetics = compute_forward_kinetics(node, node.rotation)
            idx[0] += 3

    for child in node.children:
        add_motion(child, motion_frame, idx)

def motion_adapter(root, motion_frame):
    add_motion(root, motion_frame, idx=[0])
    root_position = list(map(float, motion_frame[:3]))
    ar = root.kinetics[:3, :3]
    ap = np.array(root_position)
    ap, ar = get_pelvis_virtual(ap, ar)

    T = np.eye(4)  # 단위행렬 생성 (4x4)
    T[:3, :3] = ar  # 회전 행렬 설정
    T[:3, 3] = root.offset  # 위치 벡터 설정

    return root_position, root, ap, T

def get_rotation_matrix(channel, angle_deg):
    theta = np.deg2rad(angle_deg)
    if "Xrotation" in channel:
        return np.array([
            [1, 0, 0, 0],
            [0, np.cos(theta), -np.sin(theta), 0],
            [0, np.sin(theta),  np.cos(theta), 0],
            [0, 0, 0, 1]
        ])
    elif "Yrotation" in channel:
        return np.array([
            [ np.cos(theta), 0, np.sin(theta), 0],
            [ 0, 1, 0, 0],
            [-np.sin(theta), 0, np.cos(theta), 0],
            [ 0, 0, 0, 1]
        ])
    elif "Zrotation" in channel:
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta),  np.cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    else:
        return np.identity(4)

def translation_matrix(offset):
    tx, ty, tz = offset
    return np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])

def compute_forward_kinetics(node, rotations):
    M = translation_matrix(node.offset)
    channels = node.channels[-3:]
    if rotations is not None:
        for channel, angle in zip(channels, rotations):
            M = M @ get_rotation_matrix(channel, angle)
    return M

def get_projection(v, onto):
    onto_norm = onto / np.linalg.norm(onto)
    proj = np.dot(v, onto_norm) * onto_norm
    return proj

def lookrotation(v, u):
    u_hat = u/np.linalg.norm(u)
    v_hat = v/np.linalg.norm(v)

    vxu = np.cross(u_hat, v_hat)
    t_hat = vxu/np.linalg.norm(vxu)

    R = np.array([t_hat, np.cross(v_hat, t_hat), v_hat]).T
    return R

def get_pelvis_virtual(ap, ar):
    upVector = np.array([0,1,0], dtype=float)
    p = ap - get_projection(ap, upVector)
    f = ar[:, 2]
    r = lookrotation(f-get_projection(f, upVector), upVector)
    ap_transformed = r.T @ (ap - p)
    ar_transformed = r.T @ ar

    return ap_transformed, ar_transformed
    

