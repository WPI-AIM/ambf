from ecmIK import *
from ambf_client import Client
import time
from joint_space_trajectory_generator import JointSpaceTrajectory


def test_ik():
    # There are 7 frames for DH for the PSM (no D.O.F for 7th Joint).
    # Thus we are going to provide 7 joint values to the PSM FK which
    # gives back T_7_0 instead of T_6_0.
    js_traj = JointSpaceTrajectory(num_joints=4, num_traj_points=50)
    num_points = js_traj.get_num_traj_points()
    num_joints = 4
    for i in range(num_points):
        test_q = js_traj.get_traj_at_point(i)
        T_4_0 = compute_FK(test_q)
        computed_q = compute_IK(convert_mat_to_frame(T_4_0))
        computed_q = round_vec(computed_q, 4)

        errors = [0] * js_traj.get_num_joints()
        for j in range(num_joints):
            errors[j] = test_q[j] - computed_q[j]
        print ('--------------------------------------')
        print ('**************************************')
        print ('Test Number:', i)
        print ('Joint Positions used to T_EE_B (EndEffector in Base)')
        print test_q
        print ('IK Output')
        print computed_q
        print ('Joint Errors from IK Solver')
        print ["{0:0.2f}".format(k) for k in errors]


def test_ambf_ecm():
    c = Client('ecm_ik_test')
    c.connect()
    time.sleep(2.0)
    print(c.get_obj_names())
    b = c.get_obj_handle('ecm/baselink')
    target_ik = c.get_obj_handle('ecm/target_ik')
    target_fk = c.get_obj_handle('ecm/target_fk')
    time.sleep(1.0)

    # The following are the names of the controllable joints.
    #  'baselink-yawlink', 0
    #  'yawlink-pitchbacklink', 1
    #  'pitchendlink-maininsertionlink', 2
    #  'maininsertionlink-toollink', 3

    num_joints = 4
    joint_lims = np.zeros((num_joints, 2))
    joint_lims[0] = [np.deg2rad(-91.96), np.deg2rad(91.96)]
    joint_lims[1] = [np.deg2rad(-60), np.deg2rad(60)]
    joint_lims[2] = [0.0, 0.24]
    joint_lims[3] = [np.deg2rad(-175), np.deg2rad(175)]

    js_traj = JointSpaceTrajectory(num_joints=num_joints, num_traj_points=50, joint_limits=joint_lims)
    num_points = js_traj.get_num_traj_points()
    for i in range(num_points):

        test_q = js_traj.get_traj_at_point(i)
        T_4_0 = compute_FK(test_q)

        if target_ik is not None:
            P_0_w = Vector(b.get_pos().x, b.get_pos().y, b.get_pos().z)
            R_0_w = Rotation.RPY(b.get_rpy()[0], b.get_rpy()[1], b.get_rpy()[2])
            T_0_w = Frame(R_0_w, P_0_w)
            T_7_w = T_0_w * convert_mat_to_frame(T_4_0)
            target_ik.set_pos(T_7_w.p[0], T_7_w.p[1], T_7_w.p[2])
            target_ik.set_rpy(T_7_w.M.GetRPY()[0], T_7_w.M.GetRPY()[1], T_7_w.M.GetRPY()[2])

        computed_q = compute_IK(convert_mat_to_frame(T_4_0))
        # computed_q = enforce_limits(computed_q, joint_lims)
        
        if target_fk is not None:
            P_0_w = Vector(b.get_pos().x, b.get_pos().y, b.get_pos().z)
            R_0_w = Rotation.RPY(b.get_rpy()[0], b.get_rpy()[1], b.get_rpy()[2])
            T_0_w = Frame(R_0_w, P_0_w)
            T_4_0_fk = compute_FK(computed_q)
            T_4_w = T_0_w * convert_mat_to_frame(T_4_0_fk)
            target_fk.set_pos(T_4_w.p[0], T_4_w.p[1], T_4_w.p[2])
            target_fk.set_rpy(T_4_w.M.GetRPY()[0], T_4_w.M.GetRPY()[1], T_4_w.M.GetRPY()[2])

        # print('SETTING JOINTS: ')
        # print(computed_q)

        b.set_joint_pos('baselink-yawlink', computed_q[0])
        b.set_joint_pos('yawlink-pitchbacklink', computed_q[1])
        b.set_joint_pos('pitchendlink-maininsertionlink', computed_q[2])
        b.set_joint_pos('maininsertionlink-toollink', computed_q[3])

        test_q = round_vec(test_q)
        T_4_0 = round_mat(T_4_0, 4, 4, 3)
        errors = [0]*num_joints
        for j in range(num_joints):
            errors[j] = test_q[j] - computed_q[j]
        print ('--------------------------------------')
        print ('**************************************')
        print ('Test Number:', i)
        print ('Joint Positions used to T_EE_B (EndEffector in Base)')
        print test_q
        print ('Requested Transform for T_EE_B (EndEffector in Base)')
        print (T_4_0)
        print ('Joint Errors from IK Solver')
        print ["{0:0.2f}".format(k) for k in errors]

        time.sleep(1.0)

    time.sleep(3.0)


if __name__ == "__main__":
    # test_ik()
    test_ambf_ecm()
