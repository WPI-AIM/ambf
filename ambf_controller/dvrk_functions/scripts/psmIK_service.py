#!/usr/bin/python

NAME='compute_IK_server'

import rospy
from dvrk_functions.srv import *
from psmIK import compute_IK

def compute_IK_wrapper(req):
  return ComputeIKResponse(compute_IK(req.T_7_0))

def compute_IK_server():
  rospy.init_node(NAME)
  s = rospy.Service('compute_IK', ComputeIK, compute_IK_wrapper)

  rospy.spin()

if __name__ == '__main__':
  compute_IK_server()