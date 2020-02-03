#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool, Float32, Float64, Char, UInt64, Float64MultiArray
from decawave.msg import position
from pii.msg import trajectory
import numpy as np
import math
import matplotlib.pyplot as plt
from statistics import mean


class lane_data_reader:
    def __init__(self):
        self.time = None
        self.car_prev_x = []
        self.car_prev_y = []
        self.loc_data_x = []
        self.loc_data_y = []
        self.done = False
        self.count = 166

    def write_to_file(self, data):
        if self.done:
            return

        _x = data.data[0]
        _y = data.data[1]
        _time = data.time
        if ((not _x) or (not _y) or (abs(_x)>100) or (abs(_y)>100)):
            print("bad point")
            return
        
        self.car_prev_x.append(data.data[0])
        if len(self.car_prev_x) >= 3:
            self.loc_data_x.append(mean(self.car_prev_x))
            self.car_prev_x = []

        self.car_prev_y.append(data.data[1])
        if len(self.car_prev_y) >= 3:
            self.loc_data_y.append(mean(self.car_prev_y))
            self.car_prev_y = []

        self.count -= 1
        if self.count <= 0:
            data = np.column_stack((np.array(self.loc_data_x), np.array(self.loc_data_y)))
            np.savetxt('sparse_lane_data.dat', data)
            self.done = True
            print("Done!")
            plt.plot(self.loc_data_x, self.loc_data_y)
            plt.show()
            return 


if __name__ == '__main__':
    data_loader = lane_data_reader()
    rospy.init_node('Lane_location_loader', anonymous=True)
    rospy.Subscriber("/gem_location", position, data_loader.write_to_file)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.1)