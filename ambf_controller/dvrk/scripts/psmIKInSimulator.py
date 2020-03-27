from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math
import rospy

from psmFK import *
from psmIK import *

# Import the Client from ambf_client package
from ambf_client import Client
import time

#if __name__ == "__main__":
##    test_ik(-0.1, -0.1, -0.3, 0, PI_2, PI/4)

# Create a instance of the client
#_client = Client()

## Connect the client which in turn creates callable objects from ROS topics
## and initiates a shared pool of threads for bi-directional communication
#_client.connect()

## You can print the names of objects found. We should see all the links found
#print(_client.get_obj_names())

## Lets get a handle to PSM and ECM, as we can see in the printed
## object names, 'ecm/baselink' and 'psm/baselink' should exist
#psm_handle = _client.get_obj_handle('psm/baselink')

#time.sleep(0.2)

#psm_num_joints = psm_handle.get_num_joints()
#print(psm_num_joints)

#psm_joint_names = psm_handle.get_joint_names()
#print(psm_joint_names)

#_client.clean_up()

class PSM:
    def __init__(self):
        self._client = Client()
        self._connect = self._client.connect()
        self.psm_base_handle = self._client.get_obj_handle('psm/baselink')
        time.sleep(0.01)
        # Below are list of moveable joints
        self.j1 = 'baselink-yawlink'
        self.j2 = 'yawlink-pitchbacklink'
        self.j3 = 'pitchendlink-maininsertionlink'
        self.j4 = 'maininsertionlink-toolrolllink'
        self.j5 = 'toolrolllink-toolpitchlink'
        self.j6 = 'toolpitchlink-toolgripper1link'

#        print(self.psm_base_handle.get_joint_names())

    def set_pose(self, t1, t2, t3, t4, t5, t6):
        self.psm_base_handle.set_joint_pos(self.j1, t1)
        self.psm_base_handle.set_joint_pos(self.j2, t2)
        self.psm_base_handle.set_joint_pos(self.j3, t3)
        self.psm_base_handle.set_joint_pos(self.j4, t4)
        self.psm_base_handle.set_joint_pos(self.j5, t5)
        self.psm_base_handle.set_joint_pos(self.j6, t6)
#        time.sleep(10)


    def get_pose(self):
        j_0_1 = self.psm_base_handle.get_joint_pos(1)
#        j_0_2 = self.psm_base_handle.get_joint_pos(self.j2)
#        j_0_3 = self.psm_base_handle.get_joint_pos(self.j3)
#        j_0_4 = self.psm_base_handle.get_joint_pos(self.j4)
#        j_0_5 = self.psm_base_handle.get_joint_pos(self.j5)
#        j_0_6 = self.psm_base_handle.get_joint_pos(self.j6)

#        return j_0_1, j_0_2, j_0_3, j_0_4, j_0_5, j_0_6
        return j_0_1

    def client_clean_up(self):
        self._client.clean_up()


def set_common_frame(x, y, z, rx, ry, rz):
    Rx = Rotation.RPY(rx, 0.0, 0.0)
    Ry = Rotation.RPY(0.0, ry, 0.0)
    Rz = Rotation.RPY(0.0, 0.0, rz)

    tip_offset_rot = Rotation.RPY(np.pi, 0, np.pi/2)
    req_rot = tip_offset_rot * Rz * Ry * Rx
    req_pos = Vector(x, y, z)

    T_7_0 = Frame(req_rot, req_pos)

    return T_7_0



if __name__ == "__main__":

    T_7_0 = set_common_frame(x=-0.1, y=-0.1, z=-0.3, rx=0, ry=PI_2, rz=PI/4)

    # print "REQ POSE \n", round_transform(convert_frame_to_mat(T_7_0), 3), "\n\n--------\n\n"
    t1, t2, t3, t4, t5, t6 = compute_IK(T_7_0)

#    print(t1, t2, t3, t4, t5, t6)
    psm = PSM()
    psm.set_pose(t1, t2, t3, t4, t5, t6)

    psm.get_pose()
    j_0_1 = psm.get_pose()
    print(j_0_1)
#    j_0_1, j_0_2, j_0_3, j_0_4, j_0_5, j_0_6 = psm.get_pose()
#    print(j_0_1, j_0_2, j_0_3, j_0_4, j_0_5, j_0_6)

    psm.client_clean_up()
