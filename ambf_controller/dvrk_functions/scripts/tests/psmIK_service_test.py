#!/usr/bin/python3
import rospy
from dvrk_functions.srv import *
import time
from psmFK import *
from dvrk_functions.msg import HomogenousTransform
import numpy as np

def test_ik():
    # We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
    # in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)
    test_q = [-0.3, 0.2, 0.1, -0.9, 0.0, 0.0, 0.0]
    t_7_0 = compute_FK(test_q)
    T_7_0 = np.array(t_7_0)
    msg = HomogenousTransform()
    msg.data = T_7_0.flatten()
    print('Homogenous Transform Message:\n{}'.format(T_7_0))
    try:
        compute_IK_service = rospy.ServiceProxy('compute_IK', ComputeIK)
        compute_IK_resp = compute_IK_service.call(ComputeIKRequest(msg)) # convert_mat_to_frame(desired_end_effector_frame)
        computed_q = list(compute_IK_resp.q_des)
        for i in range(0, 6):
          computed_q[i] = round(computed_q[i], 2)
        print('Test Q: ', test_q[0:6])
        print('Comp Q: ', computed_q)
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    test_ik()
