from ambf_msgs.msg import ObjectState, ObjectCmd
import rospy
from geometry_msgs.msg import Pose
from PyKDL import Rotation, Frame, Vector

global openhmd_state, occulus_state, occulus_state_valid, openhmd_state_valid

openhmd_state = Pose()
occulus_state = ObjectState()
occulus_cmd = ObjectCmd()
occulus_state_valid = False
openhmd_state_valid = False


def openhmd_cb(msg):
    global openhmd_state, openhmd_state_valid
    openhmd_state = msg
    openhmd_state_valid = True


def occulus_cb(msg):
    global occulus_state, occulus_state_valid
    occulus_state = msg
    occulus_state_valid = True


def main():
    global openhmd_state, occulus_state, openhmd_state_valid, occulus_state_valid
    rospy.init_node('occulus_view')
    openhmd_sub = rospy.Subscriber("/openhmd/pose", Pose, openhmd_cb)
    occulus_sub = rospy.Subscriber("/ambf/env/Occulus/State", ObjectState, occulus_cb, queue_size=10)
    occulus_pub = rospy.Publisher("/ambf/env/Occulus/Command", ObjectCmd, queue_size=10)

    rate = rospy.Rate(60)
    counter = 0
    start = rospy.get_time()
    _first = True
    openhmd_initial_rot = Rotation()
    occulus_initial_rot = Rotation()
    R_pre = Rotation()
    R_aInr_offset = Rotation().RPY(0, -1.57079, -1.57079)
    # open
    while not rospy.is_shutdown():
        if openhmd_state_valid and occulus_state_valid:
            if _first:
                _first = False
                openhmd_initial_rot = Rotation.Quaternion(openhmd_state.orientation.x,
                                                          openhmd_state.orientation.y,
                                                          openhmd_state.orientation.z,
                                                          openhmd_state.orientation.w)

                occulus_initial_rot = Rotation.Quaternion(occulus_state.pose.orientation.x,
                                                          occulus_state.pose.orientation.y,
                                                          occulus_state.pose.orientation.z,
                                                          occulus_state.pose.orientation.w)

                R_pre = openhmd_initial_rot * R_aInr_offset * occulus_initial_rot.Inverse()

            else:
                occulus_cmd.pose.position = occulus_state.pose.position
                openhmd_rot = Rotation.Quaternion(openhmd_state.orientation.x,
                                                  openhmd_state.orientation.y,
                                                  openhmd_state.orientation.z,
                                                  openhmd_state.orientation.w)
                delta_rot = R_pre.Inverse() * openhmd_rot * R_aInr_offset
                # delta_rot = openhmd_rot
                occulus_cmd.pose.orientation.x = delta_rot.GetQuaternion()[0]
                occulus_cmd.pose.orientation.y = delta_rot.GetQuaternion()[1]
                occulus_cmd.pose.orientation.z = delta_rot.GetQuaternion()[2]
                occulus_cmd.pose.orientation.w = delta_rot.GetQuaternion()[3]
                occulus_cmd.enable_position_controller = True

        occulus_pub.publish(occulus_cmd)
        counter = counter + 1
        if counter % 60 == 0:
            print "- Publishing Occulus Pose ", format( round(rospy.get_time() - start, 3)), 's'
        rate.sleep()


if __name__ == "__main__":
    main()
