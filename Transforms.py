# transforms.py
import numpy as np

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
            [0, 0, 0, 1]
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
    channels = node.channels[-3:]  # Assuming last three channels are rotation channels.
    if rotations is not None:
        for channel, angle in zip(channels, rotations):
            M = M @ get_rotation_matrix(channel, angle)
    return M

def extract_yaw_rotation(kinetics):
    forward = kinetics[:3, 2].copy()
    forward[1] = 0  # Project onto XZ plane.
    norm = np.linalg.norm(forward)
    if norm != 0:
        forward /= norm
    else:
        forward = np.array([0, 0, 1])
    # Compute yaw; using negative to invert if needed.
    yaw = -np.arctan2(forward[0], forward[2])
    cos_y = np.cos(yaw)
    sin_y = np.sin(yaw)
    rotation_y = np.array([
        [cos_y, 0, sin_y, 0],
        [0, 1, 0, 0],
        [-sin_y, 0, cos_y, 0],
        [0, 0, 0, 1]
    ], dtype=float)
    return rotation_y

def motion_adapter(root, motion_frame):
    add_motion(root, motion_frame, idx=[0])
    root_position = list(map(float, motion_frame[:3]))

    return root_position, root

def add_motion(node, motion_frame, idx=[0]):
    if not node:
        return

    if node.name != "Site":
        if len(node.channels) == 6:
            # For a node with six channels, the first three are position, the next three are rotation.
            idx[0] += 3
            rotation = list(map(float, motion_frame[idx[0]:idx[0] + 3]))
            node.kinetics = compute_forward_kinetics(node, rotation)
            idx[0] += 3
        elif len(node.channels) == 3:
            rotation = list(map(float, motion_frame[idx[0]:idx[0] + 3]))
            node.kinetics = compute_forward_kinetics(node, rotation)
            idx[0] += 3

    for child in node.children:
        add_motion(child, motion_frame, idx)