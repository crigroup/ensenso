#!/usr/bin/env python
import os
import copy
import rospy
import numpy as np
import dynamic_reconfigure.client
# OpenCV and PCL
import cv2
from cv_bridge import CvBridge, CvBridgeError
try:
  import pcl
except ImportError:
  raise Exception('pcl python biddings not found: https://github.com/strawlab/python-pcl')
# Messages
from sensor_msgs.msg import (
  Image, 
  PointCloud2
)


class Snatcher(object):
  """
  Class to 'snatch' images and cloud from the ensenso camera. It subscribes to the camera 
  topics and connects to the dynamic reconfiguration server to start/stop streaming and
  to switch on/off the projector and frontlight.
  """
  def __init__(self, use_cv_types=True):
    """
    Snatcher constructor. It subscribes to the following topics:
      - Raw images: C{left/image_raw} and C{left/image_raw} of type C{sensor_msgs/Image}
      - Rectified images: C{left/image_rect} and C{left/image_rect} of type C{sensor_msgs/Image}
      - Point cloud: C{depth/points} of type C{sensor_msgs/PointCloud2}
    @type  use_cv_types: bool
    @param use_cv_types: If true will convert image messages to valid OpenCV type.
    @note: If your B{topics are different} use ros remapping.
    """
    self.initialized = False
    # Config stuff
    self.use_cv_types = use_cv_types
    self.bridge = CvBridge()
    self.cv_type = 'mono8'
    self.exposure_time = 1.5
    # Setup publishers and subscribers
    self.reset_snapshots()
    rospy.Subscriber('left/image_raw', Image, self.cb_raw_left, queue_size=1)
    rospy.Subscriber('right/image_raw', Image, self.cb_raw_right, queue_size=1)
    rospy.Subscriber('left/image_rect', Image, self.cb_rect_left, queue_size=1)
    rospy.Subscriber('right/image_rect', Image, self.cb_rect_right, queue_size=1)
    rospy.Subscriber('depth/points', PointCloud2, self.cb_point_cloud, queue_size=1)
    # Camera configuration client
    self.dynclient = dynamic_reconfigure.client.Client('ensenso_driver', timeout=30, config_callback=self.cb_dynresponse)
    self.initialized = True
  
  def cb_dynresponse(self, config):
    """
    TODO: Check that the configuration succeeded.
    """
    pass
  
  def cb_point_cloud(self, msg):
    """
    Callback executed every time a point cloud is received
    @type  msg: sensor_msgs/PointCloud2
    @param msg: The C{PointCloud2} message.
    """
    self.point_cloud = msg
    self.headers['point_cloud'] = copy.deepcopy(msg.header)
  
  def cb_raw_left(self, msg):
    """
    Callback executed every time a left raw image is received
    @type  msg: sensor_msgs/Image
    @param msg: The C{Image} message.
    """
    self.headers['raw_left'] = copy.deepcopy(msg.header)
    if self.use_cv_types:
      try:
        self.raw_left = self.bridge.imgmsg_to_cv2(msg, self.cv_type)
      except:
        rospy.logdebug('Failed to process raw_left image')
        self.raw_left = None
    else:
      self.raw_left = msg
  
  def cb_raw_right(self, msg):
    """
    Callback executed every time a right raw image is received
    @type  msg: sensor_msgs/Image
    @param msg: The C{Image} message.
    """
    self.headers['raw_right'] = copy.deepcopy(msg.header)
    if self.use_cv_types:
      try:
        self.raw_right = self.bridge.imgmsg_to_cv2(msg, self.cv_type)
      except:
        rospy.logdebug('Failed to process raw_right image')
        self.raw_right = None
    else:
      self.raw_right = msg
  
  def cb_rect_left(self, msg):
    """
    Callback executed every time a left rectified image is received
    @type  msg: sensor_msgs/Image
    @param msg: The C{Image} message.
    """
    self.headers['rect_left'] = copy.deepcopy(msg.header)
    if self.use_cv_types:
      try:
        self.rect_left = self.bridge.imgmsg_to_cv2(msg, self.cv_type)
      except:
        rospy.logdebug('Failed to process rect_left image')
        self.rect_left = None
    else:
      self.rect_left = msg
  
  def cb_rect_right(self, msg):
    """
    Callback executed every time a right raw image is received
    @type  msg: sensor_msgs/Image
    @param msg: The C{Image} message.
    """
    self.headers['rect_right'] = copy.deepcopy(msg.header)
    if self.use_cv_types:
      try:
        self.rect_right = self.bridge.imgmsg_to_cv2(msg, self.cv_type)
      except:
        rospy.logdebug('Failed to process rect_right image')
        self.rect_right = None
    else:
      self.rect_right = msg
  
  def enable_lights(self, projector=False, frontlight=False):
    """
    Switches on/off the projector and/or the frontlight
    @type  projector: bool
    @param projector: Switch on/off the projector
    @type  frontlight: bool
    @param frontlight: Switch on/off the frontlight
    """
    self.dynclient.update_configuration({'Projector':projector, 'FrontLight':frontlight})
  
  def enable_streaming(self, cloud=False, images=False):
    """
    Enable/disable the streaming of the point cloud and/or the images
    @type  cloud: bool
    @param cloud: Enable/disable the streaming of the point cloud
    @type  images: bool
    @param images: Enable/disable the streaming of the images
    """
    self.dynclient.update_configuration({'Cloud':cloud, 'Images':images})
  
  def execute(self):
    """
    Virtual method where to put the overloaded code.
    """
    raise Exception('Unimplemented method. Please overload it.')
  
  def has_cloud(self):
    """
    Checks if we snatched a point cloud
    @rtype: bool
    @return: True if successful, false otherwise
    """
    return (self.point_cloud is not None)
  
  def has_images(self):
    """
    Checks if we snatched the raw and rect images
    @rtype: bool
    @return: True if successful, false otherwise
    """
    has_raw = (self.raw_left is not None) and (self.raw_right is not None)
    has_rect =(self.rect_left is not None) and (self.rect_right is not None)
    return (has_raw and has_rect)
  
  def has_images_and_cloud(self):
    """
    Checks if we snatched the point cloud, the raw and rect images
    @rtype: bool
    @return: True if successful, false otherwise
    """
    return ( self.has_cloud() and self.has_images() )
  
  def reset_snapshots(self):
    """
    Resets the snatched information
    """
    self.headers = dict()
    self.point_cloud = None
    self.raw_left = None
    self.raw_right = None
    self.rect_left = None
    self.rect_right = None
  
  def take_snapshot(self, exposure_time, success_fn, check_interval=1/30.):
    """
    Wait until we have snatched the information encoded in C{success_fn}
    @rtype: bool
    @return: True if successful, false otherwise
    """
    exposure_time = max( 0.001, exposure_time )
    rospy.sleep(exposure_time)
    self.reset_snapshots()
    while not success_fn():
      rospy.sleep(check_interval)
      if rospy.is_shutdown():
        return False
    return True
