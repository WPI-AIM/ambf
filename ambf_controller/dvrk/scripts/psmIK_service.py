#!/usr/bin/python2

NAME = 'compute_IK_server'

import rospy
from dvrk_ambf_extensions.srv import *
from psmIK import compute_IK
from utilities import convert_mat_to_frame
import numpy as np


def compute_IK_wrapper(req):
  Transform = np.array(req.T_7_0.data).reshape((4, 4), order='C')
  # print 'Incoming Homogenous Transform Message: \n{}'.format(Transform)
  return ComputeIKResponse(compute_IK(convert_mat_to_frame(Transform)))


def compute_IK_server():
  rospy.init_node(NAME)
  s = rospy.Service('compute_IK', ComputeIK, compute_IK_wrapper)

  rospy.spin()


if __name__ == '__main__':
  compute_IK_server()