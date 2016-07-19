#!/usr/bin/env python
import os
import rospy
import argparse
import datetime
import numpy as np
import dynamic_reconfigure.client
# OpenCV and PCL
from cv_bridge import CvBridge, CvBridgeError
from ros_numpy.point_cloud2 import get_xyz_points, pointcloud2_to_array
import cv2
import pcl
# Messages
from sensor_msgs.msg import (
  Image, 
  PointCloud2
)


class Snapshotter(object):
  def __init__(self, snapshots):
    # Config stuff
    np.set_printoptions(precision=4, suppress=True)
    self.bridge = CvBridge()
    self.snapshots = snapshots
    self.exposure_time = 1.5
    # Setup publishers and subscribers
    self.reset_snapshots()
    rospy.Subscriber('left/image_raw', Image, self.cb_raw_left)
    rospy.Subscriber('right/image_raw', Image, self.cb_raw_right)
    rospy.Subscriber('left/image_rect', Image, self.cb_rect_left)
    rospy.Subscriber('right/image_rect', Image, self.cb_rect_right)
    rospy.Subscriber('depth/points', PointCloud2, self.cb_point_cloud)
    # Create folder where we will save the snapshoots
    now = rospy.get_time()
    stamp = datetime.datetime.fromtimestamp(now).strftime('%Y-%m-%d-%H-%M-%S')
    basepath = os.path.expanduser('~/.ros/snapshooter/')
    self.folder = os.path.join(basepath, stamp)
    try:
      os.makedirs(self.folder)
    except OSError, e:
      pass    # The folder already exist
    # Camera configuration client
    self.dynclient = dynamic_reconfigure.client.Client('ensenso_driver', timeout=30, config_callback=self.cb_dynresponse)
  
  def cb_dynresponse(self, config):
    # TODO: Check that the configuration succeeded.
    pass
  
  def cb_point_cloud(self, msg):
    try:
      self.point_cloud = msg
    except:
      self.point_cloud = None
  
  def cb_raw_left(self, msg):
    try:
      self.raw_left = self.bridge.imgmsg_to_cv2(msg, 'mono8')
    except:
      rospy.logdebug('Failed to process left image')
      self.raw_left = None
  
  def cb_raw_right(self, msg):
    try:
      self.raw_right = self.bridge.imgmsg_to_cv2(msg, 'mono8')
    except:
      rospy.logdebug('Failed to process right image')
      self.raw_right = None
  
  def cb_rect_left(self, msg):
    try:
      self.rect_left = self.bridge.imgmsg_to_cv2(msg, 'mono8')
    except:
      rospy.logdebug('Failed to process left image')
      self.rect_left = None
  
  def cb_rect_right(self, msg):
    try:
      self.rect_right = self.bridge.imgmsg_to_cv2(msg, 'mono8')
    except:
      rospy.logdebug('Failed to process right image')
      self.rect_right = None
  
  def execute(self):
    taken = 0
    while taken < self.snapshots and not rospy.is_shutdown():
      rospy.loginfo('Taking snapshot {0}/{1}'.format(taken+1, self.snapshots))
      # Save snapshots
      rospy.logdebug('Taking cloud snapshot')
      self.take_cloud_snapshot()
      rospy.logdebug('Saving cloud snapshot')
      self.save_cloud(taken)
      rospy.logdebug('Taking images snapshot')
      self.take_images_snapshot()
      rospy.logdebug('Saving images snapshot')
      self.save_images(taken)
      # Count
      taken += 1
    # Stop streaming
    self.dynclient.update_configuration({'Cloud':False, 'Images':False})
    rospy.loginfo('Saved {0} snapshots to: {1}'.format(taken, self.folder))
  
  def reset_snapshots(self):
    self.point_cloud = None
    self.raw_left = None
    self.raw_right = None
    self.rect_left = None
    self.rect_right = None
  
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
  
  def take_cloud_snapshot(self):
    # Switch-on projector and stream point cloud
    self.dynclient.update_configuration({'Cloud':True, 'Images':False, 'Projector':True})
    rospy.sleep(self.exposure_time)
    self.reset_snapshots()
    while self.point_cloud is None:
      rospy.sleep(0.1)
      if rospy.is_shutdown():
        return False
    return True
  
  def take_images_snapshot(self):
    # Stream only the images
    self.dynclient.update_configuration({'Cloud':False, 'Images':True, 'Projector':False})
    rospy.sleep(self.exposure_time)
    self.reset_snapshots()
    while (self.raw_left is None) or (self.raw_right is None) or (self.rect_left is None) or (self.rect_right is None):
      rospy.sleep(0.1)
      rospy.logdebug('Waiting for images')
      if rospy.is_shutdown():
        return False
    return True


if __name__ == '__main__':
  # Parse command line arguments
  parser = argparse.ArgumentParser(description='Snapshooter script')
  parser.add_argument('--snapshots', type=int, default=1, help='Number of snapshots to be taken, (default=1)')
  parser.add_argument('--debug', action='store_true',     help='If set, will show additional debugging messages')
  args = parser.parse_args(rospy.myargv()[1:])
  log_level= rospy.DEBUG if args.debug else rospy.INFO
  
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name, log_level=log_level)
  rospy.loginfo('Starting node: {0}'.format(node_name))
  shotter = Snapshotter(snapshots=args.snapshots)
  shotter.execute()
  
