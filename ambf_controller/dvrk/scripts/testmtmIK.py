from mtmIK import *
from ambf_client import Client
import time
from joint_space_trajectory_generator import JointSpaceTrajectory


def test_ik():
    js_traj = JointSpaceTrajectory(num_joints=7, num_traj_points=50)
    num_points = js_traj.get_num_traj_points()
    num_joints = 7
    for i in range(num_points):
        test_q = js_traj.get_traj_at_point(i)
        T_7_0 = compute_FK(test_q)
        computed_q = compute_IK(convert_mat_to_frame(T_7_0))
        computed_q = round_vec(computed_q, 4)

        errors = [0] * js_traj.get_num_joints()
        for j in range(num_joints):
            errors[j] = test_q[j] - computed_q[j]
        print ('******')
        print ('Case:', i)
        print ('Joint Errors from IK Solver')
        print ["{0:0.2f}".format(k) for k in errors]


def test_ambf_mtm():
    c = Client('mtm_ik_test')
    c.connect()
    time.sleep(2.0)
    print(c.get_obj_names())
    b = c.get_obj_handle('mtm/TopPanel')
    target_ik = c.get_obj_handle('mtm/target_ik')
    target_fk = c.get_obj_handle('mtm/target_fk')
    time.sleep(1.0)

    # The following are the names of the controllable joints.
    #  'TopPanel-OutPitchShoulder', 0
    #  'OutPitchShoulder-ArmParallel', 1
    #  'ArmParallel-BottomArm', 2
    #  'BottomArm-WristPlatform', 3
    #  'WristPlatform-WristPitch', 4
    #  'WristPitch-WristYaw', 5
    #  'WristYaw-WristRoll', 6

    num_joints = 7
    joint_lims = np.zeros((num_joints, 2))
    joint_lims[0] = [np.deg2rad(-75), np.deg2rad(44.98)]
    joint_lims[1] = [np.deg2rad(-44.98), np.deg2rad(44.98)]
    joint_lims[2] = [np.deg2rad(-44.98), np.deg2rad(44.98)]
    joint_lims[3] = [np.deg2rad(-59), np.deg2rad(245)]
    joint_lims[4] = [np.deg2rad(-90), np.deg2rad(180)]
    joint_lims[5] = [np.deg2rad(-44.98), np.deg2rad(44.98)]
    joint_lims[6] = [np.deg2rad(-175), np.deg2rad(175)]

    js_traj = JointSpaceTrajectory(num_joints=7, num_traj_points=50, joint_limits=joint_lims)
    num_points = js_traj.get_num_traj_points()
    num_joints = 7
    for i in range(num_points):
        test_q = js_traj.get_traj_at_point(i)
        T_7_0 = compute_FK(test_q)

        if target_ik is not None:
            P_0_w = Vector(b.get_pos().x, b.get_pos().y, b.get_pos().z)
            R_0_w = Rotation.RPY(b.get_rpy()[0], b.get_rpy()[1], b.get_rpy()[2])
            T_0_w = Frame(R_0_w, P_0_w)
            T_7_w = T_0_w * convert_mat_to_frame(T_7_0)
            target_ik.set_pos(T_7_w.p[0], T_7_w.p[1], T_7_w.p[2])
            target_ik.set_rpy(T_7_w.M.GetRPY()[0], T_7_w.M.GetRPY()[1], T_7_w.M.GetRPY()[2])

        computed_q = compute_IK(convert_mat_to_frame(T_7_0))
        # computed_q = enforce_limits(computed_q, joint_lims)

        if target_fk is not None:
            P_0_w = Vector(b.get_pos().x, b.get_pos().y, b.get_pos().z)
            R_0_w = Rotation.RPY(b.get_rpy()[0], b.get_rpy()[1], b.get_rpy()[2])
            T_0_w = Frame(R_0_w, P_0_w)
            T_7_0_fk = compute_FK(computed_q)
            T_7_w = T_0_w * convert_mat_to_frame(T_7_0_fk)
            target_fk.set_pos(T_7_w.p[0], T_7_w.p[1], T_7_w.p[2])
            target_fk.set_rpy(T_7_w.M.GetRPY()[0], T_7_w.M.GetRPY()[1], T_7_w.M.GetRPY()[2])

        # print('SETTING JOINTS: ')
        # print(computed_q)

        b.set_joint_pos('TopPanel-OutPitchShoulder', computed_q[0])
        b.set_joint_pos('OutPitchShoulder-ArmParallel', computed_q[1])
        b.set_joint_pos('ArmParallel-BottomArm', computed_q[2])
        b.set_joint_pos('BottomArm-WristPlatform', computed_q[3])
        b.set_joint_pos('WristPlatform-WristPitch', computed_q[4])
        b.set_joint_pos('WristPitch-WristYaw', computed_q[5])
        b.set_joint_pos('WristYaw-WristRoll', computed_q[6])

        test_q = round_vec(test_q)
        T_7_0 = round_mat(T_7_0, 4, 4, 3)
        errors = [0]*num_joints
        print ('--------------------------------------')
        print ('**************************************')
        print ('Test Number:', i)
        print ('Joint Positions used to T_EE_B (EndEffector in Base)')
        print test_q
        print ('Requested Transform for T_EE_B (EndEffector in Base)')
        print (T_7_0)
        print ('Joint Errors from IK Solver')
        print ["{0:0.2f}".format(k) for k in errors]

        time.sleep(1.0)

    time.sleep(3.0)


if __name__ == "__main__":
    # test_ik()
    test_ambf_mtm()
