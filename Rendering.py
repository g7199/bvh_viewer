from OpenGL.GL import *
from pyglm import glm
import numpy as np
from utils import draw_colored_cube, draw_colored_sphere, bone_rotation
from Transforms import updateJointTransform
joint_size = 3

def draw_humanoid(root, motion, frame):
    """
    Skeleton을 그리기 위한 함수입니다.
    :param root_position: skeleton을 그리기 시작할 position
    :param root_joint: 그릴 joint
    """
    frame_data = motion.frameData[frame]
    updateJointTransform(root, frame_data)
    glPushMatrix()

    glMultMatrixf(glm_mat4_to_glf(root.transform))
    glTranslatef(*frame_data.position['hip'])
    draw_joint(root.children[0])
    glPopMatrix()


def draw_joint(joint):
    """
    Joint를 그리기 위한 함수입니다.
    전역 좌표계의 kinematics를 그대로 적용하고, 관절이면 sphere, 아니라면 뼈대를 그립니다.
    :param color: RGB 컬러
    """
    glPushMatrix()
    glMultMatrixf(glm_mat4_to_glf(joint.transform))

    if joint.name != "joint_Root":
        draw_colored_sphere(joint_size)

    for child in joint.children:
        if joint.name != "joint_Root":
            draw_bone(child.offset)
        draw_joint(child)

    glPopMatrix()

def draw_bone(offset):
    """
    Skeleton에서 뼈를 그리기 위한 함수입니다.
    :param offset: 뼈 길이를 구하기 위한 값
    """
    mid = [offset[0] / 2.0, offset[1] / 2.0, offset[2] / 2.0]
    rot_quat = bone_rotation(glm.vec3(*offset))
    rot_mat = glm.mat4_cast(rot_quat)
    glPushMatrix()
    glTranslatef(*mid)
    glMultMatrixf(glm_mat4_to_glf(rot_mat))
    bone_length = glm.length(glm.vec3(*offset))
    glScalef(joint_size, abs(bone_length - 2 * joint_size) / 2, joint_size / 3)
    draw_colored_cube(1)
    glPopMatrix()

def draw_virtual_root_axis(root, axis_length=10.0):
    """
    root Transform T에서 조그만한 3차원 축을 그리기 위함입니다.
    virtual root의 위치를 받아 rotation만큼 회전하여 그려 pelvis의 회전을 시각적으로 확인할 수 있습니다.
    :param virtual_root: 축을 그릴 root
    :param rotation: 적용할 회전값
    :param axis_length: 축 크기 (기본값 10)
    """
    glPushMatrix()
    glMultMatrixf(glm_mat4_to_glf(root.transform))
    glBegin(GL_LINES)
    # X-axis in red.
    glColor3f(1, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(axis_length, 0, 0)
    # Y-axis in green.
    glColor3f(0, 1, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, axis_length, 0)
    # Z-axis in blue.
    glColor3f(0, 0, 1)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, axis_length)
    glEnd()
    glPopMatrix()



def glm_mat4_to_glf(m: glm.mat4) -> np.ndarray:
    return np.array(m.to_list(), dtype=np.float32).flatten()