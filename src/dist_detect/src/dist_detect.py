#!/usr/bin/env python
import sys
import matplotlib
import numpy as np
import projection
import rospy
from iarc_msgs.msg import Pwm, Pose, RoiPos

from dji_sdk.msg import GlobalPosition


class dist_detect(object):
    def __init__(self):
        intrinsic = np.loadtxt(
            "/home/gjx/dancer-workspace/workspaces/core/src/dconfig/matlab/bot2_in.txt"
        )
        fx = intrinsic[0]
        fy = intrinsic[1]
        cx = intrinsic[2]
        cy = intrinsic[3]
        self.camMatrix = np.mat([[fx, 0, cx, 0], \
                        [0, fy, cy, 0], \
                        [0, 0, 1, 0], \
                        [0, 0, 0, 1]])
        self.yaw = None
        self.pitch = None
        self.roll = None
        self.u = None
        self.v = None
        self.width = None
        self.height = None
        self.ifdetect = None
        self.height = None

        rospy.Subscriber('Pwm', Pwm, self.gimbal_update)
        rospy.Subscriber('/RoiPose', RoiPos, self.pos_update)
        rospy.Subscriber('dji_sdk/global_position', GlobalPosition,
                         self.height_update)
        self.dist_pub = rospy.Publisher('/Goal_global_pos',
                                        Pose,
                                        queue_size=10)

    def gimbal_update(self, pwm):
        self.roll = pwm.roll
        self.pitch = pwm.pitch
        self.yaw = pwm.yaw

    def pos_update(self, pos):
        self.u = pos.x
        self.v = pos.y
        self.width = pos.width
        self.height = pos.height
        self.ifdetect = pos.detectornot

    def height_update(self, msg):
        self.height = msg.height

    def loop(self):
        if self.ifdetect == 1:
            gpos = projection.projection(self.u, self.v, self.roll, self.pitch,
                                         self.yaw)
            pos = Pose()
            pos.x = gpos[0, 0]
            pos.y = gpos[0, 1]
            pos.z = gpos[0, 2]
            pos.distance = self.height / pos.z * np.sqrt(pos.x**2 + pos.y**2)
            self.dist_pub.publish(pos)


if __name__ == "__main__":
    rospy.init_node('dist_detect', anonymous=True)
    # rate = rospy.Rate(30)
    detect = dist_detect()
    try:
        detect.loop()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
