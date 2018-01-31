#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from genpy.rostime import Time
from rosbag.bag import Bag


def callback_rgb(data, time, bag):
    print 'received PointCloud2!'
    data.header.stamp = time + rospy.Duration.from_sec(1505388514)   # shift time stamp to the year 2017 (for kalibr)
    bag.write('/ensenso/depth/points', data, t=data.header.stamp)

def rgb_listener(bag):
    counter = 0
    while(True):
        raw_input("press enter to continue...")
        counter += 1
        time = rospy.Time(counter)
        msg_rgb = rospy.wait_for_message('/ensenso/depth/points', PointCloud2)
        callback_rgb(msg_rgb, time,bag)

if __name__ == '__main__':
    rospy.init_node('rgb_listener', anonymous=True)
    try:
        bag = Bag('images_for_calibration.bag', 'w', allow_unindexed=True)
        rgb_listener(bag)
    finally:
        bag.close()
