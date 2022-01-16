#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
from math import atan, sin, cos, degrees, radians


class ObstacleFeasibility:

    def __init__(self):
        self.X1 = 0  # psd1L or psd1R
        self.X1o = 0.92  # Flat reading of X1
        self.X1T = 0.02  # Buffer

        self.X2 = 0  # psd2L or psd2R
        self.X2o = 0.76  # Flat reading of X1
        self.X2T = 0.02  # Buffer

        self.alpha = 45  # Square Plate Tilt
        self.L = 0.2  # Distance between psd1L to psd2L, or psd1R to psd2R

        self.psd1L_sub = rospy.Subscriber("/psd1L", Range, self.psd1L_callback)
        self.psd2L_sub = rospy.Subscriber("/psd2L", Range, self.psd2L_callback)

    def psd1L_callback(self, msg):
        self.X1 = round(msg.range, 2)

    def psd2L_callback(self, msg):
        self.X2 = round(msg.range, 2)

    def check_flat_floor(self):
        if abs(self.X1 - self.X1o) <= self.X1T and abs(self.X2 - self.X2o) <= self.X2T:
            return True
        return False

    def check_downward_step(self):
        if self.X1 - self.X1o > self.X1T and self.X2 - self.X2o > self.X2T \
                and abs(self.X1 - self.X1o) - self.X1T < abs(self.X2 - self.X2o) + self.X2T:
            step_height = (self.X2o - self.X2) * sin(radians(self.alpha))
            return True, step_height
        return False, 0

    def check_upward_step(self):
        if self.X1 - self.X1o <= -self.X1T and self.X2 - self.X2o <= -self.X2T \
                and abs(self.X1 - self.X1o) - self.X1T < abs(self.X2 - self.X2o) + self.X2T \
                or (abs(self.X1 - self.X1o) <= self.X1T and self.X2 - self.X2o <= -self.X2T):
            step_height = (self.X2o - self.X2) * sin(radians(self.alpha))
            return True, step_height
        return False, 0

    def check_trench(self):
        trench_depth = 0
        trench_width = []
        is_trench = False

        while abs(self.X1 - self.X1o) <= self.X1T and self.X2 - self.X2o > self.X2T:
            trench_depth = (self.X2o - self.X2) * sin(radians(self.alpha))
            trench_width.append((self.X2o - self.X2) * cos(radians(self.alpha)))
            is_trench = True

        if not trench_width:
            return is_trench, trench_depth, 0
        return is_trench, trench_depth, max(trench_width)

    def check_downward_slope(self):
        if abs(self.X1 - self.X1o) > self.X1T and abs(self.X2 - self.X2o) > self.X2T \
                and self.X1 - self.X1o - self.X1T > self.X2 - self.X2o + self.X2T:
            slope_angle = 90 - self.alpha - degrees(atan((self.X1 - self.X2) / self.L))
            return True, slope_angle
        return False, 0

    def check_upward_slope(self):
        if abs(self.X1 - self.X1o) > self.X1T and abs(self.X2 - self.X2o) > self.X2T \
                and self.X1 - self.X1o - self.X1T < self.X2 - self.X2o + self.X2T:
            slope_angle = 90 - self.alpha - degrees(atan((self.X1 - self.X2) / self.L))
            return True, slope_angle
        return False, 0


def calculate():
    ob1 = ObstacleFeasibility()

    while True:
        if ob1.check_flat_floor():
            print("FLAT")
        else:
            downward_step, step_height = ob1.check_downward_step()
            if downward_step:
                print("DOWNWARD STEP = %sm" % step_height)

            upward_step, step_height = ob1.check_upward_step()
            if upward_step:
                print("UPWARD STEP = %sm" % step_height)

            is_trench, trench_depth, trench_width = ob1.check_trench()
            if is_trench:
                print("TRENCH DEPTH = %sm, TRENCH WIDTH = %sm" % (trench_depth, trench_width))

            downward_slope, slope_angle = ob1.check_downward_slope()
            if downward_slope:
                print("DOWNWARD SLOPE = %s degrees" % slope_angle)

            upward_slope, slope_angle = ob1.check_upward_slope()
            if upward_slope:
                print("UPWARD SLOPE = %s degrees" % slope_angle)


if __name__ == '__main__':
    try:

        rospy.init_node('Communication', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)

        calculate()

    except rospy.ROSInterruptException:
        pass
