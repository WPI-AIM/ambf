from psmIK import *


def test_ik(x, y, z, rx, ry, rz):
    # Rx = Rotation.RPY(rx, 0.0, 0.0)
    # Ry = Rotation.RPY(0.0, ry, 0.0)
    # Rz = Rotation.RPY(0.0, 0.0, rz)
    #
    # tip_offset_rot = Rotation.RPY(np.pi, 0, np.pi/2)
    # req_rot = tip_offset_rot * Rz * Ry * Rx
    # req_pos = Vector(x, y, z)
    # T_7_0 = Frame(req_rot, req_pos)
    # # print "REQ POSE \n", round_transform(convert_frame_to_mat(T_7_0), 3), "\n\n--------\n\n"
    # compute_IK(T_7_0)

    # We are going to provide 7 joint values to the PSM FK, the 7th value is ignore for FK purposes but results
    # in the FK returning us T_7_0 rather than T_6_0. There 7 frame from DH is a fixed frame (no D.O.F)
    test_q = [0.5, -0.9, 0.1, 0.5, 0.6, -0.9, 0.0]
    T_7_0 = compute_FK(test_q)
    computed_q = compute_IK(convert_mat_to_frame(T_7_0))
    for i in range(0, 6):
        computed_q[i] = round(computed_q[i], 4)

    print('Test Q: ', test_q[0:6])
    print('Comp Q: ', computed_q)


if __name__ == "__main__":
    test_ik(-0.1, -0.1, -0.3, 0, PI_2, PI/4)