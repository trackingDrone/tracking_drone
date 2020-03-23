#!/usr/bin/python3
from transform import *


def calc_extrinsic(rollRad, pitchRad, yawRad):
    # world to gimbal, R means rotate
    Zb2c = 0.1
    b2c = rotateZ(yawRad) \
        * rotateY(pitchRad) \
        * rotateX(rollRad) \
        * dtranslate(0,0,-Zb2c)

    # temp = [0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1]
    # temp = np.array(temp).reshape(4, 4)
    extrinsic = b2c
    return extrinsic
