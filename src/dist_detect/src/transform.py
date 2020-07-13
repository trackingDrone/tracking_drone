import numpy as np


def rotateX(rx):
    # https://en.wikipedia.org/wiki/Rotation_matrix
    R = np.mat([[1, 0, 0, 0], [0, np.cos(rx), -np.sin(rx), 0],
                [0, np.sin(rx), np.cos(rx), 0], [0, 0, 0, 1]])
    return R


def rotateY(ry):
    # https://en.wikipedia.org/wiki/Rotation_matrix
    R = np.mat([[np.cos(ry), 0, np.sin(ry), 0], [0, 1, 0, 0],
                [-np.sin(ry), 0, np.cos(ry), 0], [0, 0, 0, 1]])
    return R


def rotateZ(rz):
    # https://en.wikipedia.org/wiki/Rotation_matrix
    R = np.mat([[np.cos(rz), -np.sin(rz), 0, 0],
                [np.sin(rz), np.cos(rz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return R


def dtranslate(dx, dy, dz):
    T = np.mat([[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]])
    return T