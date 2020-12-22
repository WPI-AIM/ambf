#!/usr/bin/env python

import sys
import PyKDL as p
from frame import Frame
from vector import Vector, dot
from rotation import Rotation
import numpy as np
import math

PI = np.pi
PI_2 = np.pi/2


# The up vector is useful to define angles > PI. Since otherwise
# this method will only report angles <= PI.
def py_kdl_get_angle(vec_a, vec_b, up_vector=None):
    vec_a.Normalize()
    vec_b.Normalize()
    cross_ab = vec_a * vec_b
    vdot = p.dot(vec_a, vec_b)
    # print('VDOT', vdot, vec_a, vec_b)
    # Check if the vectors are in the same direction
    if 1.0 - vdot < 0.000001:
        angle = 0.0
        # Or in the opposite direction
    elif 1.0 + vdot < 0.000001:
        angle = np.pi
    else:
        angle = math.acos(vdot)

    if up_vector is not None:
        same_dir = np.sign(dot(cross_ab, up_vector))
        if same_dir < 0.0:
            angle = -angle

    return angle

# The up vector is useful to define angles > PI. Since otherwise
# this method will only report angles <= PI.
def tf_utils_get_angle(vec_a, vec_b, up_vector=None):
    vec_a.Normalize()
    vec_b.Normalize()
    cross_ab = vec_a * vec_b
    vdot = dot(vec_a, vec_b)
    # print('VDOT', vdot, vec_a, vec_b)
    # Check if the vectors are in the same direction
    if 1.0 - vdot < 0.000001:
        angle = 0.0
        # Or in the opposite direction
    elif 1.0 + vdot < 0.000001:
        angle = np.pi
    else:
        angle = math.acos(vdot)

    if up_vector is not None:
        same_dir = np.sign(dot(cross_ab, up_vector))
        if same_dir < 0.0:
            angle = -angle

    return angle

def get_angle(vec_a, vec_b, up_vector=None):
    # pykdl_angle = py_kdl_get_angle(vec_a, vec_b, up_vector)
    tf_utils_angle = tf_utils_get_angle(vec_a, vec_b, up_vector)

    # assert pykdl_angle == tf_utils_angle, 'Angles do not match'

    return tf_utils_angle


def round_mat(mat, rows, cols, precision=4):
    for i in range(0, rows):
        for j in range(0, cols):
            mat[i, j] = round(mat[i, j], precision)
    return mat


def round_vec(vec, precision=4):
    for i in range(3):
        vec[i] = round(vec[i], precision)
    return vec


def round_transform(mat, precision=4):
    return round_mat(mat, 4, 4, precision)


def convert_frame_to_mat(frame):
    np_mat = np.mat([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)
    for i in range(3):
        for j in range(3):
            np_mat[i, j] = frame.M[(i, j)]

    for i in range(3):
        np_mat[i, 3] = frame.p[i]

    return np_mat


def pykdl_convert_mat_to_frame(mat):
    frame = p.Frame(p.Rotation.RPY(0, 0, 0), p.Vector(0, 0, 0))
    for i in range(3):
        for j in range(3):
            frame[(i, j)] = mat[i, j]

    for i in range(3):
        frame.p[i] = mat[i, 3]

    return frame

def tf_utils_convert_mat_to_frame(mat):
    frame = Frame(Rotation.RPY(0, 0, 0), Vector(0, 0, 0))
    for i in range(3):
        for j in range(3):
            frame.M[i, j] = mat[i, j]

    for i in range(3):
        frame.p[i] = mat[i, 3]

    return frame

def convert_mat_to_frame(mat):
    pykdl_frame = pykdl_convert_mat_to_frame(mat)
    tf_utils_frame = tf_utils_convert_mat_to_frame(mat)

    # a = np.asarray(pykdl_frame.M)
    # assert np.all(a, tf_utils_frame.M), 'Rotations do not match'
    # assert np.all(pykdl_frame.p, tf_utils_frame.p), 'Points do not match'

    return tf_utils_frame