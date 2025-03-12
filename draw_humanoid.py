from pyglm import glm
import numpy as np
from functions import *
from utils import draw_undercircle
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

joint_size = 3

from utils import *

def draw_humanoid(root_position, root_joint, ap, T):
    glPushMatrix()
    glTranslatef(*root_position)
    draw_joint(root_joint, ap, T, True)
    glPopMatrix()

def draw_joint(joint, ap, T, root):
    glPushMatrix()
    glMultMatrixf(joint.kinetics.T.flatten())

    if root:
        glPushMatrix()
        glMultMatrixf(T.flatten())
        glTranslatef(0, -joint.position[1], 0)
        draw_virtual(ap, T)
        glPopMatrix()

    if joint.name != "joint_Root":
        draw_colored_sphere(joint_size)

    for child in joint.children:
        glPushMatrix()
        if joint.name != "joint_Root":
            draw_bone(child.offset)
        draw_joint(child, None, None, False)
        glPopMatrix()
    glPopMatrix()

def draw_bone(offset):
    mid = [offset[0] / 2.0, offset[1] / 2.0, offset[2] / 2.0]
    rot_quat = bone_rotation(glm.vec3(*offset))
    rot_mat = glm.mat4_cast(rot_quat)

    glPushMatrix()
    glTranslatef(*mid)
    glMultMatrixf(np.array(rot_mat, dtype=np.float32).flatten())
    glScalef(joint_size, abs(glm.l2Norm(offset) - 2 * joint_size) / 2, joint_size/3)
    draw_colored_cube(1)
    glPopMatrix()

def draw_virtual(ap, T, circle_radius=10, arrow_length=20):
    glPushMatrix()
    glTranslatef(ap[0], 0, ap[2])
    draw_arrow(T, circle_radius, arrow_length)
    glRotatef(90, 1.0, 0.0, 0.0)
    glColor3f(1.0, 1.0, 1.0) 
    draw_undercircle(10)
    glPopMatrix()