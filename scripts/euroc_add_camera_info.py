#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import argparse
import rospy
import rosbag
import sensor_msgs.msg
import yaml
import numpy as np
from numpy.linalg import inv
import cv2
import tf


# parse camera calibration yaml file
def load_intrinsics( calib_data ):

  width, height = calib_data['resolution']
  #cam_info.distortion_model = 'plumb_bob'
  D = np.array(calib_data['distortion_coefficients'])
  #cam_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
  fu, fv, cu, cv = calib_data['intrinsics']
  K = np.array([[fu, 0, cu],
                [0, fv, cv],
                [0, 0, 1]])

  return height, width, K, D


# parse camera calibration yaml file
def load_extrinsics( calib_data ):

  # read homogeneous rotation and translation matrix
  transformation_base_camera = np.array(calib_data['T_BS']['data'])
  transformation_base_camera = transformation_base_camera.reshape( (4,4) )

  # compute projection matrix
#  projection = np.zeros((3,4))
#  projection[:,:-1] = K
#  cam_info.P = projection.reshape(-1,).tolist()

  return transformation_base_camera



# create camera info message
def create_camera_info_msg(width, height, K, D, R, P):

  cam_info = sensor_msgs.msg.CameraInfo()
  cam_info.width = width
  cam_info.height = height
  cam_info.distortion_model = 'plumb_bob'
  cam_info.D = D.tolist()
  cam_info.R = R.reshape(-1,).tolist()
  cam_info.K = K.reshape(-1,).tolist()
  cam_info.P = P.reshape(-1,).tolist()

  return cam_info


if __name__ == "__main__":


  ####################################################################
  # Parse program options
  ####################################################################

  parser = argparse.ArgumentParser()

  parser.add_argument('rosbag', help='euroc rosbag file')
  parser.add_argument('left_calibration', help='euroc left camera calibration')
  parser.add_argument('right_calibration', help='euroc right camera calibration')
  parser.add_argument('--scaling', help='if set, crop/scale rectified image to region of interest (no black areas)', action="store_true")

  args = parser.parse_args()

  ####################################################################
  # Process data
  ####################################################################

  # load yaml file
  left_calib_stream = file(args.left_calibration, 'r')
  left_calib_data = yaml.load( left_calib_stream )

  right_calib_stream = file(args.right_calibration, 'r')
  right_calib_data = yaml.load( right_calib_stream )

  # parse information from calibration euroc files
  height_left, width_left, K_left, D_left = load_intrinsics( left_calib_data )
  height_right, width_right, K_right, D_right = load_intrinsics( right_calib_data )

  transformation_base_left = load_extrinsics( left_calib_data )
  transformation_base_right = load_extrinsics( right_calib_data )

  # compute transformation between left and right camera
  transformation_right_base = inv(transformation_base_right)
  transformation_right_left = transformation_right_base.dot(transformation_base_left)

  R = transformation_right_left[0:3,0:3]
  T = transformation_right_left[0:3,3]

  print('R')
  print(R)
  print('T')
  print(T)

  ####################################################################
  # Compute Rectification parameters
  ####################################################################
  R_left = np.empty([3,3])
  R_right = np.empty([3,3])
  P_left = np.empty([3,4])
  P_right = np.empty([3,4])
  # perform cropping to ROI only if specified
  alpha = 0 if args.scaling else -1
  cv2.stereoRectify(K_left, D_left, K_right, D_right, (width_left, height_left), R, T, R_left, R_right, P_left, P_right, None, cv2.CALIB_ZERO_DISPARITY, alpha)
  ####################################################################
  # Print results
  ####################################################################
  print('Intrinsic matrices (original)')
  print(K_left)
  print(K_right)
  print('Projection matrices')
  print(P_left)
  print(P_right)
  print('Rectification matrices')
  print(R_left)
  print(R_right)
  print("Distortion coefficients")
  print(D_left)
  print(D_right)


  left_camera_info_msg = create_camera_info_msg(width_left, height_left, K_left, D_left, R_left, P_left)
  right_camera_info_msg = create_camera_info_msg(width_right, height_right, K_right, D_right, R_right, P_right)

#  num_msgs = 1000

  with rosbag.Bag('output.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag(args.rosbag).read_messages():

#      if num_msgs < 1:
#          break
#      num_msgs -= 1

      outbag.write(topic, msg, t)

      # create left camera_info
      if (topic == '/cam0/image_raw'):
        #
        left_camera_info_msg.header = msg.header
        outbag.write("/cam0/camera_info", left_camera_info_msg, t)

      # create right camera_info
      if (topic == '/cam1/image_raw'):
        right_camera_info_msg.header = msg.header
        outbag.write("/cam1/camera_info", right_camera_info_msg, t)

  # Close opend file
  outbag.close()

  ####################################################################
  # compute rigit transformation between euroc left camera (cam0) and base_link
  ####################################################################

  rotation_base_left = transformation_base_left[0:3,0:3]
  pitch, yaw, roll = tf.transformations.euler_from_matrix(transformation_base_left, axes='sxyz')
  translation_base_left = transformation_base_left[0:3,3]
  print('Yaw Pitch Roll ')
  print(yaw, pitch, roll)
  print('translation')
  print(translation_base_left)





