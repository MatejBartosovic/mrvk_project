#!/usr/bin/env python

import os, sys, time, copy
import math
import numpy as np
from threading import Thread
from ring_buffer import RingBuffer

import rospy
import tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class LaserFilter():
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        self.laser_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=10)
        
        # Dist cutoff
	self.min_cutoff_distance = rospy.get_param("~min_cutoff_distance", np.inf)

        # Isolated point filter
        self.filter_isolated = rospy.get_param('~filter_isolated', False)
        self.min_accept_size = rospy.get_param('~min_accept_size', 3)

        # Transient points filter params
        self.filter_transient = rospy.get_param('~filter_transient', True)
        self.shift_tolerance = rospy.get_param('~shift_tolerance', 2)
        self.num_prev_frames = rospy.get_param('~num_prev_frames', 3) + 1

        # Prev data ring buffer
        # self.prev_data = RingBuffer(self.num_prev_frames)
        self.prev_data = np.full((self.num_prev_frames, 50), np.inf)
        self.scan_cnt = 0 # ring buffer counter

        if self.filter_isolated:
            rospy.loginfo("Isolated points filter enabled, min accept size {}"\
                .format(self.min_accept_size))
        if self.filter_transient:
            rospy.loginfo("Transient points filter enabled, using {} prev frames"\
                .format(self.num_prev_frames-1))

    def laser_cb(self, data):
        #print len(data.ranges)
        # self.ranges_in = np.array(data.ranges)
        # print self.ranges_in.shape
        # self.ranges_in *= 0.5
        
        # if self.prev_data is not None:
        #     print(np.array(self.prev_data.ranges))
        #     print ""
        debug=False
        
        if debug:
            rospy.loginfo(np.array(data.ranges))
            # print(np.array(data.ranges))
            print ""
        
        #data = self.min_cutoff(data, self.min_cutoff_distance)
        if self.filter_isolated:
            data = self.remove_isolated_points(data, self.min_accept_size)
        if self.filter_transient:
            data = self.remove_transient_points(data, self.shift_tolerance)

        # data.ranges = self.ranges_in
        if debug:
            # print(data.ranges)
            rospy.loginfo(data.ranges)
            # print("*****************************")
            rospy.loginfo("*****************************")
        self.laser_pub.publish(data)
    
    def min_cutoff(self, data, low):
        ranges = np.array(data.ranges)
        for i in range(0, len(data.ranges)):
            if ranges[i] < low:
                ranges[i] = np.inf
        data.ranges = ranges
        #print(ranges)
        return data

    def remove_isolated_points(self, data, min_accept_size):
        count = 0
        is_region = False
        i = 0
        start_i = 0
        end_i = 0
        ranges = np.array(data.ranges)

        for i in range(0,ranges.shape[0]):
            dist = ranges[i]
            if (not np.isinf(dist)) and not (np.isnan(dist)):
                # print dist,
                count+=1
                if is_region == False:
                    is_region = True
                    start_i = i
            else:
                if is_region == True:
                    is_region = False
                    end_i = i
                    if(count<min_accept_size):
                        for j in range(start_i,end_i):
                            ranges[j] = np.inf
                    count=0

            # i+=1
        data.ranges = ranges
        return data

    def remove_transient_points(self, new_data, tolerance):
        # convert scan to numpy array
        new_data.ranges = np.array(new_data.ranges)

        # if number of measurements doesn't match, re-build the buffer
        if(self.prev_data.shape[1]!=new_data.ranges.shape[0]):
            rospy.logwarn("Scan size changed from {} to {}, rebuilding buffer" \
                .format(self.prev_data.shape[1], new_data.ranges.shape[0]))
            self.prev_data = np.full((self.num_prev_frames, new_data.ranges.shape[0]), np.inf)

        # add current scan to buffer
        self.prev_data[self.scan_cnt] = copy.deepcopy(new_data.ranges)
        # print self.prev_data
        # print np.isfinite(self.prev_data)

        # calculate boolean validity mask
        # i-th point is valid, if all i-th points in all previous frames are valid
        # valid point is set to its value from the latest scan, otherwise to infinity 
        bmask = np.logical_and.reduce(np.isfinite(self.prev_data))
        for i in range(0,new_data.ranges.shape[0]):
            if not bmask[i]:
                new_data.ranges[i] = np.inf

        # print bmask
        # print new_data.ranges
        # print ""
        # if not self.prev_data.data: # is None:
        #     self.prev_data.append(new_data)
        # new_data_clean = copy.deepcopy(new_data)

        # # new_ranges = np.array(new_data.ranges)
        # for i in range(0, new_data.ranges.shape[0]):
        #     is_valid_point = True
        #     for prev in self.prev_data.get():
        #         is_valid_point &= not (self.is_invalid(prev.ranges[i]) and not self.is_invalid(new_data.ranges[i]))
        #         # if self.is_invalid(prev.ranges[i]) and not self.is_invalid(new_data.ranges[i]):
        #         if not is_valid_point:
        #             # if pixel is the same as previous
        #             new_data.ranges[i] = np.inf
        #         # else:
        #         #     # check pixels wihtin tolerance
        #         #     for s in range(0, self.shift_tolerance):

        #     #     pass
        #     # if (isnan(self.prev_data.ranges[i]) or isinf(self.prev_data.ranges[i]))
        #     #     and not 

        # self.prev_data.append(new_data_clean)
        # self.print_buffer(self.prev_data)

        self.scan_cnt = (self.scan_cnt+1)%self.num_prev_frames # update ring buffer counter
        return new_data

    def is_invalid(self, val):
        return np.isinf(val) or np.isnan(val)

    def print_buffer(self, buf):
        for data in buf.get():
            print [ "{:0.2f}".format(x) for x in data.ranges ]
        print ""


def main():	
    # print("foo")
    # print(sys.version)
    rospy.init_node('mrvk_laser_filter_node', anonymous=True)
    rospy.loginfo('Starting mrvk_laser_filter node')
    #foo
    lf = LaserFilter()

    rospy.spin()



if __name__ == '__main__':
    main()
