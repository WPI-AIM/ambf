from psmIK import *
from ambf_client import Client
import time

def test_ik():
    # We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
    # in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)
    test_q = [-0.3, 0.2, 0.1, -0.9, 0.0, 0.0, 0.0]
    T_7_0 = compute_FK(test_q)

    computed_q = compute_IK(convert_mat_to_frame(T_7_0))
    for i in range(0, 6):
        computed_q[i] = round(computed_q[i], 4)

    print('Test Q: ', test_q[0:6])
    print('Comp Q: ', computed_q)


def test_ambf_psm():
    c = Client()
    c.connect()
    time.sleep(2.0)
    print(c.get_obj_names())
    b = c.get_obj_handle('baselink')
    time.sleep(1.0)

    # The following are the names of the controllable joints.
    #  'baselink-yawlink', 0
    #  'yawlink-pitchbacklink', 1
    #  'pitchendlink-maininsertionlink', 2
    #  'maininsertionlink-toolrolllink', 3
    #  'toolrolllink-toolpitchlink', 4
    #  'toolpitchlink-toolgripper1link', 5a
    #  'toolpitchlink-toolgripper2link', 5b

    test_q = [-0.3, 0.2, 0.1, -0.9, 0.0, -1.2, 0.0]
    T_7_0 = compute_FK(test_q)
    computed_q = compute_IK(convert_mat_to_frame(T_7_0))

    print('SETTING JOINTS: ')
    print(computed_q)

    b.set_joint_pos('baselink-yawlink', computed_q[0])
    b.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
    b.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
    b.set_joint_pos('maininsertionlink-toolrolllink', computed_q[3])
    b.set_joint_pos('toolrolllink-toolpitchlink', computed_q[4])
    b.set_joint_pos('toolpitchlink-toolgripper1link', computed_q[5])
    b.set_joint_pos('toolpitchlink-toolgripper2link', -computed_q[5])

    time.sleep(3.0)


if __name__ == "__main__":
    # test_ik()
    test_ambf_psm()
