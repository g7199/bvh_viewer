# transforms.py
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

def get_rotation_matrix(channel, angle_deg):
    """
    채널에 존재하는 각도를 회전 행렬로 바꿔주는 함수입니다.
    :param channel: 바꿀 채널
    :param angle_deg: 각도 input
    :return: 회전 행렬
    """
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
    """
    Translation (x,y,z)를 행렬로 바꿔주는 함수입니다.
    :param offset: (x,y,z) translation vector
    :return: translation 행렬
    """
    tx, ty, tz = offset
    return np.array([
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])

def compute_forward_kinetics(node, rotations):
    """
    각 joint별로 rotation을 받아와 local하게 kinemetic을 구현하기 위한 행렬입니다.
    Joint의 offset에 저장되어 있는 x,y,z를 translation 행렬로 적용합니다.
    :param node: 적용할 Node (Joint)
    :param rotations: rotation 값
    :return: Forward Kinetic을 적용한 4x4 행렬
    """
    M = translation_matrix(node.offset)
    channels = node.channels[-3:]  # 마지막 채널 3개가 rotation값
    if rotations is not None:
        for channel, angle in zip(channels, rotations):
            M = M @ get_rotation_matrix(channel, angle)
    return M

def extract_yaw_rotation(kinetics):
    """
    회전행렬에서 yaw값만을 추출하기 위한 함수입니다.
    XZ평면에 사영된 root Transform T 에 y축 회전을 적용하느넫 사용됩니다.
    :param kinetics: 적용되고 있는 회전
    :return: yaw값만을 담고있는 4x4 행렬
    """
    forward = kinetics[:3, 2].copy()
    forward[1] = 0  # Project onto XZ plane.
    norm = np.linalg.norm(forward)
    if norm != 0:
        forward /= norm
    else:
        forward = np.array([0, 0, 1])

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
    """
    root를 목표로 motion을 적용시키기 위한 함수입니다.
    add_motion 함수를 콜해 적용시키고 root position과 root를 return합니다.
    :param root: 적용할 root
    :param motion_frame: 모션 프레임값
    :return: root_position과 root를 return
    """
    add_motion(root, motion_frame, idx=[0])
    root_position = list(map(float, motion_frame[:3]))

    return root_position, root

def add_motion(node, motion_frame, idx=[0]):
    """
    모션을 root로부터 각 joint의 kinetics에 local하게 재귀적으로 더해주는 함수입니다.
    :param node: 적용할 node, 재귀적으로 작동한다.
    :param motion_frame: 모션프레임
    :param idx: 인덱스
    """
    if not node:
        return

    if node.name != "Site":
        if len(node.channels) == 6:
            # 첫 3개가 position 다음 3개가 rotation
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

def inverse_matrix(T):
    """
    역행렬 구해주는 함수
    :param T: 역행렬을 구할 행렬
    :return: T의 역행렬
    """
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = R.T
    t_inv = -R_inv @ t
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

def interpolate_frames(frame_a, frame_b, blend):
    """
    두 motion frame 사이를 Lerp를 사용해 interpolate하는 함수입니다.
    :param frame_a: 첫번째 motion frame (list)
    :param frame_b: 두번째 motion frame (list)
    :param blend: Blend factor, 0이면 a, 1이면 b로 linear하게
    :return: Blend된 motion
    """
    a = list(map(float, frame_a))
    b = list(map(float, frame_b))
    blended_frame = [(1 - blend) * a_val + blend * b_val for a_val, b_val in zip(a, b)]
    return blended_frame


def interpolate_frames_with_quat(frame_a, frame_b, t, root_joint):
    """
    Interpolates two motion frames using SLERP for rotation and LERP for position.
    """
    a = list(map(float, frame_a))
    b = list(map(float, frame_b))
    blended = []
    idx = 0

    def slerp_angles(euler_a, euler_b):
        key_times = [0, 1]
        key_rots = R.from_euler('xyz', [euler_a, euler_b], degrees=True)
        slerp = Slerp(key_times, key_rots)
        rot_interp = slerp([t])[0]
        return rot_interp.as_euler('xyz', degrees=True)

    def blend_joint(joint):
        nonlocal idx
        joint_channels = joint.channels
        joint_values = []

        if 'position' in ''.join(joint_channels).lower():
            for i in range(3):
                joint_values.append((1 - t) * a[idx] + t * b[idx])
                idx += 1

        if 'rotation' in ''.join(joint_channels).lower():
            rot_a = [a[idx + i] for i in range(3)]
            rot_b = [b[idx + i] for i in range(3)]
            rot_interp = slerp_angles(rot_a, rot_b)
            joint_values.extend(rot_interp)
            idx += 3

        blended.extend(joint_values)
        for child in joint.children:
            blend_joint(child)

    blend_joint(root_joint.children[0])
    return blended