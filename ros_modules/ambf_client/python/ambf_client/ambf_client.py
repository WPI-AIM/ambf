# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020, AMBF
#     (https://github.com/WPI-AIM/ambf)
#
#     All rights reserved.
#
#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:
#
#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#
#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.
#
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================

# ROS version
import os
from ros_abstraction_layer import ral

import threading
import time
from difflib import SequenceMatcher

from ambf_msgs.msg import ActuatorState, ActuatorCmd
from ambf_msgs.msg import CameraState, CameraCmd
from ambf_msgs.msg import LightState, LightCmd
from ambf_msgs.msg import ObjectState, ObjectCmd
from ambf_msgs.msg import RigidBodyState, RigidBodyCmd
from ambf_msgs.msg import GhostObjectState,GhostObjectCmd
from ambf_msgs.msg import WorldState, WorldCmd
from ambf_msgs.msg import SensorState, SensorCmd, ContactSensorState, ContactSensorCmd
from ambf_msgs.msg import VehicleState, VehicleCmd

from std_msgs.msg import Empty
from geometry_msgs.msg import WrenchStamped
from difflib import SequenceMatcher

from .ambf_actuator import Actuator
from .ambf_camera import Camera
from .ambf_light import Light
from .ambf_object import Object
from .ambf_rigid_body import RigidBody
from .ambf_sensor import Sensor, ContactSensor
from .ambf_ghost_object import GhostObject
from .ambf_vehicle import Vehicle
from .ambf_world import World


class Client:
    def __init__(self, client_name='ambf_client'):
        self._ros_topics = []
        self._sub_list = []
        self._objects_dict = {}
        self._pub_thread = []
        self._world_name = ''
        self._common_obj_namespace = ''
        self._client_name = client_name
        self._world_handle = None
        self._rate = None
        self.ral = None
        self._executor = None
        self._sub_thread = None

    def set_publish_rate(self, rate):
        self._rate = self.ral.create_rate(rate)

    def create_objs_from_rostopics(self, publish_rate):
        self.ral = ral(self._client_name)
        self._ros_topics = self.ral.get_published_topics()
        self.set_publish_rate(publish_rate)


        # Find the common longest substring to make the object names shorter
        first_run = True
        for i in range(len(self._ros_topics)):
            topic_name = self._ros_topics[i][0]
            msg_type = self._ros_topics[i][1].replace('/msg/', '/') # For ROS 2 with adds /msg/

            if msg_type in ['ambf_msgs/ActuatorState',
                            'ambf_msgs/CameraState',
                            'ambf_msgs/LightState',
                            'ambf_msgs/ObjectState',
                            'ambf_msgs/RigidBodyState',
                            'ambf_msgs/GhostObjectState',
                            'ambf_msgs/SensorState',
                            'ambf_msgs/ContactSensorState',
                            'ambf_msgs/VehicletState']:
                if first_run:
                    first_run = False
                    self._common_obj_namespace = topic_name
                else:
                    seq_match = SequenceMatcher(None, self._common_obj_namespace, topic_name)
                    match = seq_match.find_longest_match(0, len(self._common_obj_namespace), 0, len(topic_name))
                    if match.size != 0 and match.a == 0:
                        self._common_obj_namespace = self._common_obj_namespace[match.a: match.a + match.size]
                    else:
                        print('No common object namespace found, aborting search')
                        self._common_obj_namespace = ''
                        break
        print('Found Common Object Namespace as: ', self._common_obj_namespace)

        for i in range(len(self._ros_topics)):
            topic_name = self._ros_topics[i][0]
            msg_type = self._ros_topics[i][1].replace('/msg/', '/') # For ROS 2 with adds /msg/
            if msg_type == 'ambf_msgs/WorldState':
                self._world_name = 'World'
                world_obj = World(self._world_name, self.ral)
                world_obj._sub = self.ral.subscriber(topic_name, WorldState, world_obj.ros_cb)
                world_obj._pub = self.ral.publisher(topic_name.replace('/State', '/Command'), WorldCmd)
                world_obj._reset_pub = self.ral.publisher(topic_name.replace('/State', '/Command/Reset'), Empty, queue_size = 1)
                world_obj._reset_bodies_pub = self.ral.publisher(topic_name.replace('/State', '/Command/Reset/Bodies'), Empty, queue_size = 1)
                self._world_handle = world_obj
                self._objects_dict[world_obj.get_name()] = world_obj
            elif msg_type == 'ambf_msgs/ActuatorState':
                # pre_trimmed_name = topic_niyme.replace(self._common_obj_namespace, '')
                post_trimmed_name = topic_name.replace('/State', '')
                base_obj = Actuator(ral = self.ral, a_name = post_trimmed_name)
                base_obj._state = ActuatorState()
                base_obj._cmd = ActuatorCmd()
                base_obj._sub = self.ral.subscriber(topic_name, ActuatorState, base_obj.ros_cb)
                base_obj._pub = self.ral.publisher(topic_name.replace('/State', '/Command'), ActuatorCmd)
                self._objects_dict[base_obj.get_name()] = base_obj
            elif msg_type == 'ambf_msgs/CameraState':
                # pre_trimmed_name = topic_niyme.replace(self._common_obj_namespace, '')
                post_trimmed_name = topic_name.replace('/State', '')
                base_obj = Camera(ral = self.ral, a_name = post_trimmed_name)
                base_obj._state = CameraState()
                base_obj._cmd = CameraCmd()
                base_obj._sub = self.ral.subscriber(topic_name, CameraState, base_obj.ros_cb)
                base_obj._pub = self.ral.publisher(topic_name.replace('/State', '/Command'), CameraCmd)
                self._objects_dict[base_obj.get_name()] = base_obj
            elif msg_type == 'ambf_msgs/LightState':
                # pre_trimmed_name = topic_niyme.replace(self._common_obj_namespace, '')
                post_trimmed_name = topic_name.replace('/State', '')
                base_obj = Light(ral = self.ral, a_name = post_trimmed_name)
                base_obj._state = LightState()
                base_obj._cmd = LightCmd()
                base_obj._sub = self.ral.subscriber(topic_name, LightState, base_obj.ros_cb)
                base_obj._pub = self.ral.publisher(topic_name.replace('/State', '/Command'), LightCmd)
                self._objects_dict[base_obj.get_name()] = base_obj
            elif msg_type == 'ambf_msgs/ObjectState':
                # pre_trimmed_name = topic_niyme.replace(self._common_obj_namespace, '')
                post_trimmed_name = topic_name.replace('/State', '')
                base_obj = Object(ral = self.ral, a_name = post_trimmed_name)
                base_obj._state = ObjectState()
                base_obj._cmd = ObjectCmd()
                base_obj._sub = self.ral.subscriber(topic_name, ObjectState, base_obj.ros_cb)
                base_obj._pub = self.ral.publisher(topic_name.replace('/State', '/Command'))
                self._objects_dict[base_obj.get_name()] = base_obj
            elif msg_type == 'ambf_msgs/RigidBodyState':
                # pre_trimmed_name = topic_niyme.replace(self._common_obj_namespace, '')
                post_trimmed_name = topic_name.replace('/State', '')
                base_obj = RigidBody(ral = self.ral, a_name = post_trimmed_name)
                base_obj._state = RigidBodyState()
                base_obj._cmd = RigidBodyCmd()
                base_obj._sub = self.ral.subscriber(topic_name, RigidBodyState, base_obj.ros_cb)
                base_obj._pub = self.ral.publisher(topic_name.replace('/State', '/Command'), RigidBodyCmd)
                self._objects_dict[base_obj.get_name()] = base_obj
            elif msg_type == 'ambf_msgs/GhostObjectState':
                # pre_trimmed_name = topic_niyme.replace(self._common_obj_namespace, '')
                post_trimmed_name = topic_name.replace('/State', '')
                base_obj = GhostObject(ral = self.ral, a_name = post_trimmed_name)
                base_obj._state = GhostObjectState()
                base_obj._cmd = GhostObjectCmd()
                base_obj._sub = self.ral.subscriber(topic_name, GhostObjectState, base_obj.ros_cb)
                base_obj._pub = self.ral.publisher(topic_name.replace('/State', '/Command'), GhostObjectCmd)
                self._objects_dict[base_obj.get_name()] = base_obj
            elif msg_type == 'ambf_msgs/SensorState':
                # pre_trimmed_name = topic_niyme.replace(self._common_obj_namespace, '')
                post_trimmed_name = topic_name.replace('/State', '')
                base_obj = Sensor(ral = self.ral, a_name = post_trimmed_name)
                base_obj._state = SensorState()
                base_obj._cmd = SensorCmd()
                base_obj._sub = self.ral.subscriber(topic_name, SensorState, base_obj.ros_cb)
                base_obj._pub = self.ral.publisher(topic_name.replace('/State', '/Command'), SensorCmd)
                self._objects_dict[base_obj.get_name()] = base_obj
            elif msg_type == 'ambf_msgs/ContactSensorState':
                # pre_trimmed_name = topic_niyme.replace(self._common_obj_namespace, '')
                post_trimmed_name = topic_name.replace('/State', '')
                base_obj = ContactSensor(ral = self.ral, name = post_trimmed_name)
                base_obj._state = ContactSensorState()
                base_obj._cmd = ContactSensorCmd()
                base_obj._sub = self.ral.subscriber(topic_name, ContactSensorState, base_obj.ros_cb)
                base_obj._pub = self.ral.publisher(topic_name.replace('/State', '/Command'), ContactSensorCmd)
                self._objects_dict[base_obj.get_name()] = base_obj
            elif msg_type == 'ambf_msgs/VehicleState':
                # pre_trimmed_name = topic_niyme.replace(self._common_obj_namespace, '')
                post_trimmed_name = topic_name.replace('/State', '')
                base_obj = Vehicle(ral = self.ral, a_name = post_trimmed_name)
                base_obj._state = VehicleState()
                base_obj._cmd = VehicleCmd()
                base_obj._sub = self.ral.subscriber(topic_name, VehicleState, base_obj.ros_cb)
                base_obj._pub = self.ral.publisher(topic_name.replace('/State', '/Command'), VehicleCmd)
                self._objects_dict[base_obj.get_name()] = base_obj

        self.ral.spin()

    def connect(self, default_publish_rate = 120):
        self.create_objs_from_rostopics(publish_rate = default_publish_rate)
        self.start()

    def refresh(self):
        self.clean_up()
        self.connect()

    def start(self):
        self._start_pubs()

    def get_common_namespace(self):
        return self._common_obj_namespace

    def get_world_handle(self):
        return self._world_handle

    def get_obj_names(self):
        obj_names = []
        for key, obj in self._objects_dict.items():
            obj_names.append(obj.get_name())
        return obj_names

    def get_obj_handle(self, a_name):
        found_obj = None
        obj = self._objects_dict.get(a_name)
        if obj:
            found_obj = obj
        else:
            # Try matching the object name to existing names with the closest match
            objects = []
            for key, item in self._objects_dict.items():
                if key.find(a_name) >= 0:
                    objects.append(item)

            if len(objects) == 1:
                found_obj = objects[0]
            elif len(objects) == 0:
                print(a_name, 'NAMED OBJECT NOT FOUND')
                found_obj = None
            elif len(objects) > 1:
                print('WARNING FOUND ', len(objects), 'WITH MATCHING NAME:')
                for i in range(len(objects)):
                    print(objects[i].get_name())
                print('PLEASE SPECIFY FULL NAME TO GET THE OBJECT HANDLE')
                found_obj = None

        if type(found_obj) == Object:
            found_obj.set_active()
            found_obj.set_publish_children_names_flag(True)
            found_obj.set_publish_joint_names_flag(True)
            found_obj.set_publish_joint_positions_flag(True)
        elif type(found_obj) == RigidBody:
            found_obj.set_active()
            found_obj.set_publish_children_names_flag(True)
            found_obj.set_publish_joint_names_flag(True)
            found_obj.set_publish_joint_positions_flag(True)
        elif type(found_obj) == Actuator:
            found_obj.set_active()
        elif type(found_obj) == Camera:
            found_obj.set_active()
        elif type(found_obj) == Light:
            found_obj.set_active()
        elif type(found_obj) == Sensor:
            found_obj.set_active()
        elif type(found_obj) == Vehicle:
            found_obj.set_active()

        return found_obj

    def get_obj_pose(self, a_name):
        obj = self._objects_dict.get(a_name)
        if obj is not None:
            return obj.pose
        else:
            return None

    def set_obj_cmd(self, a_name, fx, fy, fz, nx, ny, nz):
        obj = self._objects_dict.get(a_name)
        obj.command(fx, fy, fz, nx, ny, nz)

    def _start_pubs(self):
        self._pub_thread = threading.Thread(target=self._run_obj_publishers)
        self._pub_thread.daemon = True
        self._pub_thread.start()

    def is_shutdown(self):
        return self.ral.is_shutdown()

    def get_time(self):
        return self.ral.to_sec(self.ral.now())

    def create_rate(self, rate):
        return self.ral.create_rate(rate)

    def get_ral(self):
        return self.ral

    def _run_obj_publishers(self):
        while not self.is_shutdown():
            for key, obj in self._objects_dict.items():
                if obj.is_active():
                    obj.run_publisher()
            self._rate.sleep()

    def print_active_topics(self):
        print(self._ros_topics)
        pass

    def print_summary(self):
        print('_________________________________________________________')
        print('---------------------------------------------------------')
        print('CLIENT FOR CREATING OBJECTS FROM ROSTOPICS')
        print('Searching Object names from ros topics with')
        print('Prefix: ', self._search_prefix_str)
        print('Suffix: ', self._search_suffix_str)
        print('Number of OBJECTS found', len(self._objects_dict))
        for key, value in self._objects_dict.items():
            print(key)
        print('---------------------------------------------------------')

    def clean_up(self):
        for key, val in self._objects_dict.items():
            val.pub_flag = False
            print('Closing publisher for: ', key)
        self._objects_dict.clear()
