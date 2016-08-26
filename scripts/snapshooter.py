#!/usr/bin/env python
import os
import rospy
import argparse
import datetime
import numpy as np
# OpenCV and PCL
import cv2
import pcl
try:
  import ros_numpy
except ImportError:
  raise Exception('ros_numpy package not found: https://github.com/eric-wieser/ros_numpy')
from ros_numpy.point_cloud2 import get_xyz_points, pointcloud2_to_array
# Python ensenso snatcher
from ensenso.snatcher import Snatcher

class Snapshotter(Snatcher):
  def __init__(self, snapshots, exposure_time):
    super(Snapshotter, self).__init__()
    # Config stuff
    np.set_printoptions(precision=5, suppress=True)
    self.snapshots = snapshots
    self.exposure_time = exposure_time
    # Create folder where we will save the snapshoots
    now = rospy.get_time()
    stamp = datetime.datetime.fromtimestamp(now).strftime('%Y-%m-%d-%H-%M-%S')
    basepath = os.path.expanduser('~/.ros/snapshooter/')
    self.folder = os.path.join(basepath, stamp)
    try:
      os.makedirs(self.folder)
    except OSError, e:
      pass    # The folder already exist
  
  def execute(self):
    taken = 0
    while taken < self.snapshots and not rospy.is_shutdown():
      rospy.loginfo('Taking snapshot {0}/{1}'.format(taken+1, self.snapshots))
      # Save snapshots
      rospy.logdebug('Taking cloud snapshot')
      self.enable_lights(projector=True, frontlight=False)
      self.enable_streaming(cloud=True, images=False)
      self.take_snapshot(self.exposure_time, success_fn=self.has_cloud)
      rospy.logdebug('Saving cloud snapshot')
      self.save_cloud(taken)
      rospy.logdebug('Taking images snapshot')
      self.enable_lights(projector=False, frontlight=False)
      self.enable_streaming(cloud=False, images=True)
      self.take_snapshot(self.exposure_time, success_fn=self.has_images)
      rospy.logdebug('Saving images snapshot')
      self.save_images(taken)
      # Count
      taken += 1
    # Stop streaming and lights
    self.enable_streaming(cloud=False, images=False)
    self.enable_lights(projector=False, frontlight=False)
    rospy.loginfo('Saved {0} snapshots to: {1}'.format(taken, self.folder))
  
  def save_cloud(self, seq):
    cloud_img = get_xyz_points(pointcloud2_to_array(self.point_cloud), remove_nans=False, dtype=np.float32)
    # DO NOT COPY THE POINT CLOUD ANYMORE. ALWAYS USES RESHAPED VIEWS
    cloud_xyz = cloud_img.view()
    cloud_xyz.shape = (-1,3)
    cloud = pcl.PointCloud( cloud_xyz )
    path = os.path.join(self.folder, '{:04d}_cloud.pcd'.format(seq))
    cloud.to_file(path)
  
  def save_images(self, seq):
    cv2.imwrite(os.path.join(self.folder, '{:04d}_raw_left.png'.format(seq)), np.array(np.squeeze(self.raw_left)))
    cv2.imwrite(os.path.join(self.folder, '{:04d}_raw_right.png'.format(seq)), np.array(np.squeeze(self.raw_right)))
    cv2.imwrite(os.path.join(self.folder, '{:04d}_rect_left.png'.format(seq)), np.array(np.squeeze(self.rect_left)))
    cv2.imwrite(os.path.join(self.folder, '{:04d}_rect_right.png'.format(seq)), np.array(np.squeeze(self.rect_right)))


if __name__ == '__main__':
  # Parse command line arguments
  parser = argparse.ArgumentParser(description='Snapshooter script')
  parser.add_argument('--snapshots', type=int,  default=2,    help='Number of snapshots to be taken, (default=2)')
  parser.add_argument('--exposure', type=float, default=1.0,  help='Exposure time in seconds, (default=1.0)')
  parser.add_argument('--debug', action='store_true',         help='If set, will show additional debugging messages')
  args = parser.parse_args(rospy.myargv()[1:])
  log_level= rospy.DEBUG if args.debug else rospy.INFO
  
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name, log_level=log_level)
  rospy.loginfo('Starting node: {0}'.format(node_name))
  shotter = Snapshotter(args.snapshots, args.exposure)
  shotter.execute()
  
