import rospy
from PyKDL import Frame, Vector, Rotation
from sensor_msgs.msg import Joy
from ambf_client import Client
import sys
import time
from argparse import ArgumentParser


# Init everything related to SpaceNav
class SpaceNavDevice:
    # The name should include the full qualified prefix. I.e. '/Geomagic/', or '/omniR_' etc.
    def __init__(self, name, ambf_client, obj_name, camera_obj_handle=None):
        joy_str = name + 'joy'

        self._active = False
        self._scale = 0.001
        self.grey_button_pressed = False
        self.white_button_pressed = False

        self._lin_vel = Vector(0, 0, 0)
        self._ang_vel = Vector(0, 0, 0)

        self._lin_scale = 0.01
        self._ang_scale = 0.005

        self._obj_pos_cmd = Vector(0, 0, 0)
        self._obj_rpy_cmd = Vector(0, 0, 0)

        self._ambf_cam_handle = camera_obj_handle
        self._obj_active = False # Flag to check if obj is active
        self._cmd_pos = Vector(0, 0, 0)
        self._cmd_rpy = Vector(0, 0, 0)

        self._pose_sub = rospy.Subscriber(joy_str, Joy, self.joy_cb, queue_size=10)

        print('BINDING SPACE NAV DEVICE: ', name, 'TO MOCK DEVICE: ', obj_name)

        self._obj_handle = ambf_client.get_obj_handle(obj_name)
        if self._obj_handle is None:
            print('ERROR, CAN\'T FIND ', obj_name)
            print('EXISTING OBJECTS ARE: ')
            print(ambf_client.get_obj_names())

        else:
            time.sleep(0.5)
            # self._obj_pos_cmd = self.

        self._msg_counter = 0

    def joy_cb(self, msg):

        self._lin_vel = Vector(-msg.axes[0], -msg.axes[1], msg.axes[2])
        self._ang_vel = Vector(-msg.axes[3], -msg.axes[4], msg.axes[5])

        self._active = True
        pass

    def process_commands(self):
        if self._active and self._obj_handle is not None:
            if self._obj_handle.is_wd_expired():
                self._obj_active = False

            if not self._obj_active:
                cur_pos = Vector(self._obj_handle.get_pos().x,
                                 self._obj_handle.get_pos().y,
                                 self._obj_handle.get_pos().z)

                cur_rpy = Vector(self._obj_handle.get_rpy()[0],
                                 self._obj_handle.get_rpy()[1],
                                 self._obj_handle.get_rpy()[2])

                if self._obj_handle.is_wd_expired:
                    self._obj_active = True
            else:
                cur_pos = self._cmd_pos
                cur_rpy = self._cmd_rpy

            rot = Rotation.RPY(0, 0, 0)
            if self._ambf_cam_handle is not None:
                cam_rpy = self._ambf_cam_handle.get_rpy()
                rot = Rotation.RPY(cam_rpy[0],
                                   cam_rpy[1],
                                   cam_rpy[2])

            self._cmd_pos = cur_pos + rot * (self._lin_vel * self._lin_scale)
            self._cmd_rpy = cur_rpy + (self._ang_vel * self._ang_scale)

            self._obj_handle.set_pos(self._cmd_pos[0],
                                     self._cmd_pos[1],
                                     self._cmd_pos[2])

            self._obj_handle.set_rpy(self._cmd_rpy[0],
                                     self._cmd_rpy[1],
                                     self._cmd_rpy[2])

            if self._msg_counter % 500 == 0:
                pass
            if self._msg_counter >= 1000:
                self._msg_counter = 0

            self._msg_counter = self._msg_counter + 1


def main():
    # Begin Argument Parser Code
    parser = ArgumentParser()

    parser.add_argument('-d', action='store', dest='spacenav_name', help='Specify ros base name of spacenav',
                        default='/spacenav/')
    parser.add_argument('-o', action='store', dest='obj_name', help='Specify AMBF Obj Name', default='BoxTool')
    parser.add_argument('-c', action='store', dest='camera_name', help='Specify AMBF Camera Name',
                        default='default_camera')
    parser.add_argument('-a', action='store', dest='client_name', help='Specify AMBF Client Name',
                        default='client_name')
    parser.add_argument('-p', action='store', dest='print_obj_names', help='Print Object Names',
                        default=False)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print parsed_args

    _spacenav_one_name = parsed_args.spacenav_name
    _obj_one_name = parsed_args.obj_name
    _default_camera_name = parsed_args.camera_name
    _ambf_client_name = parsed_args.client_name
    _print_obj_names = parsed_args.print_obj_names

    client = Client(_ambf_client_name)
    client.connect()

    if _print_obj_names:
        print ('Printing Found AMBF Object Names: ')
        print client.get_obj_names()
        exit()

    _pair_one_specified = True

    _device_pairs = []

    default_camera = client.get_obj_handle(_default_camera_name)

    # The publish frequency
    _pub_freq = 500

    spacenav_one = SpaceNavDevice(_spacenav_one_name, client, _obj_one_name, default_camera)
    _device_pairs.append(spacenav_one)

    rate = rospy.Rate(_pub_freq)
    msg_index = 0
    _start_time = rospy.get_time()

    while not rospy.is_shutdown():
        for dev in _device_pairs:
            dev.process_commands()

        rate.sleep()
        msg_index = msg_index + 1
        if msg_index % _pub_freq * 3 == 0:
            # Print every 3 seconds as a flag to show that this code is alive
            print('SpaceNav AMBF Node Alive...', round(rospy.get_time() - _start_time, 3), 'secs')
        if msg_index >= _pub_freq * 10:
            # After ten seconds, reset, no need to keep increasing this
            msg_index = 0


if __name__ == '__main__':
    main()
