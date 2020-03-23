#!/usr/bin/env python
#encoding = utf-8
import rospy
import time
import iarc_msgs.msg
import pigpio

def gimbal_update(pwm):
    roll_pwm=pwm.roll
    pitch_pwm=pwm.pitch
    yaw_pwm=pwm.yaw

    piPWM.set_servo_pulsewidth(16, roll_pwm)
    piPWM.set_servo_pulsewidth(20, pitch_pwm)
    piPWM.set_servo_pulsewidth(21, yaw_pwm)	


if __name__ == '__main__':
    rospy.init_node('gimbal_driver', anonymous=True) 
    piPWM = pigpio.pi();
    piPWM.set_mode(16, pigpio.OUTPUT)
    piPWM.set_mode(20, pigpio.OUTPUT)
    piPWM.set_mode(21, pigpio.OUTPUT)
    try:
        #debug_pub_pwm = rospy.Publisher('debug_sailsheet_pwm', Float32, queue_size=10)
        rospy.Subscriber('Pwm',iarc_msgs.msg.Pwm ,gimbal_update)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


