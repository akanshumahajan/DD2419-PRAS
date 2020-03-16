#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 25 11:25:09 2020

@author: Fredrik Forsberg
"""

import numpy as np


###


def get_rotational_matrix_from_rotational_vector(rot_vector, order=("x", "y", "z")):
    if len(rot_vector) != 3:
        raise ValueError('Incorrect argument for rot_vector in get_rotational_matrix_from_rotational_vector: ' +
                         str(rot_vector))

    if order != ("x", "y", "z") and list(set(order)) != ["z", "y", "x"]:
        raise ValueError('Incorrect argument for order in get_rotational_matrix_from_rotational_vector: ' + str(order))

    translation = {"z": "yaw", "y": "pitch", "x": "roll"}
    translated_order = tuple([translation[x] for x in order])

    return get_rotational_matrix_from_roll_pitch_yaw(rot_vector[0], -rot_vector[1], -rot_vector[2], 
                                                     order=translated_order)


def get_rotational_matrix_from_roll_pitch_yaw(roll, pitch, yaw, order=("roll", "pitch", "yaw")):
    # http://planning.cs.uiuc.edu/node102.html
    # Returns the rotational matrix. The middle angle in order must be +-pi/2 (+-90 degrees) while the other two angles
    # can be +-pi (+-180 degrees). All angles in radians.
    r = {"yaw": np.asarray([[np.cos(yaw), -np.sin(yaw), 0],
                            [np.sin(yaw), np.cos(yaw), 0],
                            [0, 0, 1]], dtype=np.float64),
         "pitch": np.asarray([[np.cos(pitch), 0, np.sin(pitch)],
                              [0, 1, 0],
                              [-np.sin(pitch), 0, np.cos(pitch)]], dtype=np.float64),
         "roll": np.asarray([[1, 0, 0],
                             [0, np.cos(roll), -np.sin(roll)],
                             [0, np.sin(roll), np.cos(roll)]], dtype=np.float64)}

    return r[order[2]].dot(r[order[1]]).dot(r[order[0]])


#

def get_rotational_vector_from_rotation_matrix(rotation_matrix, order=("x", "y", "z")):
    translation = {"z": "yaw", "y": "pitch", "x": "roll"}
    translated_order = tuple([translation[x] for x in order])

    roll_pitch_yaw = np.asarray(list(get_roll_pitch_yaw_from_rotation_matrix(rotation_matrix, translated_order)), 
                                dtype=np.float64)
    return np.expand_dims(roll_pitch_yaw * np.array([1, -1, -1]), axis=1)


def get_roll_pitch_yaw_from_rotation_matrix(rotation_matrix, order=("roll", "pitch", "yaw")):
    # http://planning.cs.uiuc.edu/node103.html
    # Returns yaw, pitch and roll from the rotational matrix where the middle angle (whose formula contains np.sqrt)
    #   is +-pi/2 (+-90 degrees) and the other two angles are +-pi (+-180 degrees). All angles in radians.

    if order == ("roll", "pitch", "yaw"):
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        pitch = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

    elif order == ("pitch", "roll", "yaw"):
        yaw = np.arctan2(-rotation_matrix[0, 1], rotation_matrix[1, 1])
        pitch = np.arctan2(-rotation_matrix[2, 0], rotation_matrix[2, 2])
        roll = np.arctan2(rotation_matrix[2, 1], np.sqrt(rotation_matrix[2, 0] ** 2 + rotation_matrix[2, 2] ** 2))

    elif order == ("roll", "yaw", "pitch"):
        yaw = np.arctan2(rotation_matrix[1, 0], np.sqrt(rotation_matrix[2, 0] ** 2 + rotation_matrix[0, 0] ** 2))
        pitch = np.arctan2(-rotation_matrix[2, 0], rotation_matrix[0, 0])
        roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])

    elif order == ("yaw", "roll", "pitch"):
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[1, 1])
        pitch = np.arctan2(rotation_matrix[0, 2], rotation_matrix[2, 2])
        roll = np.arctan2(-rotation_matrix[1, 2], np.sqrt(rotation_matrix[0, 2] ** 2 + rotation_matrix[2, 2] ** 2))

    elif order == ("pitch", "yaw", "roll"):
        yaw = np.arctan2(-rotation_matrix[0, 1], np.sqrt(rotation_matrix[0, 2] ** 2 + rotation_matrix[0, 0] ** 2))
        pitch = np.arctan2(rotation_matrix[0, 2], rotation_matrix[0, 0])
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[1, 1])

    elif order == ("yaw", "pitch", "roll"):
        yaw = np.arctan2(-rotation_matrix[0, 1], rotation_matrix[0, 0])
        pitch = np.arctan2(rotation_matrix[0, 2], np.sqrt(rotation_matrix[0, 1] ** 2 + rotation_matrix[0, 0] ** 2))
        roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[2, 2])

    else:
        raise ValueError('Incorrect argument for order in get_yaw_pitch_roll_from_rotation_matrix: ' + str(order))

    return roll, pitch, yaw


#


def get_rotations_from_rotation_matrix_unnecessarily_complicated_version(rotation_matrix, order=("roll", "pitch",
                                                                                                 "yaw")):
    # Unnecessarily complicated version of get_rotations_from_rotation_matrix_complicated_version
    # http://planning.cs.uiuc.edu/node102.html
    # http://planning.cs.uiuc.edu/node103.html
    # Returns yaw, pitch and roll from the rotational matrix where the middle angle (whose formula contains np.sqrt)
    #   is +-pi/2 (+-90 degrees) and the other two angles are +-pi (+-180 degrees). All angles in radians.

    r = {"yaw": np.asarray([[1, -1, 0],
                            [1, 1, 0],
                            [0, 0, 1]], dtype=np.float64),
         "pitch": np.asarray([[1, 0, 1],
                              [0, 1, 0],
                              [-1, 0, 1]], dtype=np.float64),
         "roll": np.asarray([[1, 0, 0],
                             [0, 1, -1],
                             [0, 1, 1]], dtype=np.float64)}

    dummy_sign = r[order[2]].dot(r[order[1]]).dot(r[order[0]])

    # Depending on the order, the sign array will look differently, but always have one row and one column with +-1
    # By finding the rows and columns of either the two zeros or the two twos, we can find the "central" position of
    #   the five +-1. This term is cruicial for the second type of rotation in the order.
    #
    # ('roll', 'pitch', 'yaw')    ('pitch', 'roll', 'yaw')    ('roll', 'yaw', 'pitch')
    # [[ 1.  0.  2.]              [[ 0. -1.  2.]              [[ 1.  0.  2.]
    #  [ 1.  2.  0.]               [ 2.  1.  0.]               [ 1.  1. -1.]
    #  [-1.  1.  1.]]              [-1.  1.  1.]]              [-1.  2.  0.]]
    #
    # ('yaw', 'roll', 'pitch')    ('pitch', 'yaw', 'roll')    ('yaw', 'pitch', 'roll')
    # [[ 2.  0.  1.]              [[ 1. -1.  1.]              [[ 1. -1.  1.]
    #  [ 1.  1. -1.]               [ 2.  1.  0.]               [ 2.  0. -1.]
    #  [ 0.  2.  1.]]              [ 0.  1.  2.]]              [ 0.  2.  1.]]
    #
    # By elementwise addition of the zero coordinates, we can get the missing row and column
    center = tuple([{1: 2, 2: 1, 3: 0}[x] for x in np.sum(np.argwhere(dummy_sign == 0), axis=1)])

    return_dict = {}

    for i in range(0, 3):
        if i == 0:
            coord0 = ((center[0] - 1) % 3, center[1])
            coord1 = ((center[0] - 2) % 3, center[1])
            return_dict[order[i]] = np.arctan2(rotation_matrix[coord0] * dummy_sign[coord0],
                                               rotation_matrix[coord1] * dummy_sign[coord1])
        elif i == 1:
            coord0 = ((center[0] - 1) % 3, center[1])  # Does also work with the coordinates for i=2 instead of i=0
            coord1 = ((center[0] - 2) % 3, center[1])
            return_dict[order[i]] = np.arctan2(rotation_matrix[center] * dummy_sign[center],
                                               np.sqrt(rotation_matrix[coord0] ** 2 + rotation_matrix[coord1] ** 2))
        else:
            coord0 = (center[0], (center[1] + 1) % 3)
            coord1 = (center[0], (center[1] + 2) % 3)
            return_dict[order[i]] = np.arctan2(rotation_matrix[coord0] * dummy_sign[coord0],
                                               rotation_matrix[coord1] * dummy_sign[coord1])

    return return_dict["roll"], return_dict["pitch"], return_dict["yaw"]


#


def get_rotational_matrix_from_vectors(vector0, vector1):
    # Rotational Matrix: https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
    # Input vectors are numpy 3x1 vectors with dtype np.float64

    # Check vector0
    if type(vector0) != np.ndarray:
        vector0 = np.expand_dims(np.asarray(vector0, dtype=np.float64), axis=1)
    elif len(vector0.shape) == 1:
        vector0 = np.expand_dims(vector0, axis=1)
    elif vector0.shape == (1, 3):
        vector0 = vector0.T
    if len(vector0.shape) != 2 or vector0.shape != (3, 1):
        raise ValueError('Incorrect argument vector0 in get_rotational_matrix_from_vectors: ' + str(vector0))

    # Check vector1
    if type(vector1) != np.ndarray:
        vector1 = np.expand_dims(np.asarray(vector1, dtype=np.float64), axis=1)
    elif len(vector1.shape) == 1:
        vector1 = np.expand_dims(vector1, axis=1)
    elif vector1.shape == (1, 3):
        vector1 = vector1.T
    if len(vector1.shape) != 2 or vector1.shape != (3, 1):
        raise ValueError('Incorrect argument vector1 in get_rotational_matrix_from_vectors: ' + str(vector1))

    # Normalise vectors
    try:
        vector0 = vector0 / np.linalg.norm(vector0)
    except ZeroDivisionError:
        pass
    try:
        vector1 = vector1 / np.linalg.norm(vector1)
    except ZeroDivisionError:
        pass

    rot_axis = np.cross(vector0.T, vector1.T).T
    angle = np.arccos(np.dot(vector0.T, vector1))[0][0]

    return get_rotational_matrix_from_rotation_around_vector(rot_axis, angle)


#


def get_rotational_matrix_from_rotation_around_vector(rot_axis, angle):
    # Rotational Matrix: https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
    # Input vector rot_axis should be numpy 3x1 vector with dtype np.float64
    # angle should be in radians and rotating according to the right hand rule

    # Check rot_axis
    if type(rot_axis) != np.ndarray:
        rot_axis = np.expand_dims(np.asarray(rot_axis, dtype=np.float64), axis=1)
    elif len(rot_axis.shape) == 1:
        rot_axis = np.expand_dims(rot_axis, axis=1)
    elif rot_axis.shape == (1, 3):
        rot_axis = rot_axis.T
    if len(rot_axis.shape) != 2 or rot_axis.shape != (3, 1):
        raise ValueError('Incorrect argument rot_axis in get_rotational_matrix: ' + str(rot_axis))

    # Normalise rot_axis
    try:
        rot_axis = rot_axis / np.linalg.norm(rot_axis)
    except ZeroDivisionError:
        pass

    cross_product_matrix = np.asarray([[0, -rot_axis[2], rot_axis[1]],
                                       [rot_axis[2], 0, -rot_axis[0]],
                                       [-rot_axis[1], rot_axis[0], 0]], dtype=np.float64)
    outer_product = np.dot(rot_axis, rot_axis.T)

    return (np.cos(angle) * np.identity(3, dtype=np.float64) + np.sin(angle) * cross_product_matrix +
            (1 - np.cos(angle)) * outer_product)


#


def rotate_vector_around_vector(vector, rotation_axis, angle):
    # Rodrigues' rotation formula: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula#Statement
    # Input vectors are numpy 3x1 vectors with dtype np.float64
    if np.linalg.norm(rotation_axis) != 1:
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    return (vector * np.cos(angle) + np.cross(rotation_axis, vector, axis=0) * np.sin(angle) +
            rotation_axis * np.dot(rotation_axis.T, vector) * (1 - np.cos(angle)))
