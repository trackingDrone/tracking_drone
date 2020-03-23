#!/usr/bin/python3
# from main import camMatrix
import numpy as np


def projection(u, v, roll, pitch, yaw, camMatrix):
    from calc_extrinsic import calc_extrinsic
    extrinsic = calc_extrinsic(roll, pitch, yaw)

    PointImage = extrinsic * camMatrix.I * np.mat([[u], [v], [0], [1]])

    PointImage = np.mat([
        PointImage[0, 0][0, 0], PointImage[1, 0][0, 0], PointImage[2, 0][0, 0]
    ])

    return PointImage