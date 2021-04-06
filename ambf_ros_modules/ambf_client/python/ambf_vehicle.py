#!/usr/bin/env python
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

from ambf_msgs.msg import VehicleState
from ambf_msgs.msg import VehicleCmd
from ambf_base_object import BaseObject


class Vehicle(BaseObject):
    def __init__(self, a_name, time_out=0.1):
        """
        Constructor
        :param a_name:
        """
        super(Vehicle, self).__init__(a_name, time_out)  # Set duration of Watchdog expiry
        self.object_type = "VEHICLE"
        self.body_type = "DYNAMIC"

        # The vehicle can have many wheels, set the appropriate wheel indexes which are meant to be steered
        self._steering_wheel_indices = []
        # The vehicle can have many wheels, set the appropriate wheel indexes which are meant to be powered
        self._powered_wheel_indices = []

        self._initialized = False

    def _clear_command(self):
        """
        Clear cmd if watchdog is expired
        :return:
        """

    def get_wheel_count(self):
        """
        Get the number of wheel of this vehicle
        :return:
        """
        return self._state.wheel_count

    def check_wheel_count(self, idx):
        """
        Get the number of wheel of this vehicle
        :return:
        """
        if idx >= self._state.wheel_count:
            # The idx is greater than the wheel count
            print('ERROR! IDX: ', idx, ' GREATER THAN TOTAL WHEEL COUNT OF ', self._state.wheel_count)
            return False
        else:
            return True

    def set_steered_wheel_indices(self, indices):
        # Sanity check, make sure that none of the indexes are greater than the wheel count
        for idx in indices:
            if not self.check_wheel_count(idx):
                return False

        self._steering_wheel_indices = indices
        self.initialize_wheel_cmds()

    def initialize_wheel_cmds(self):
        if len(self._cmd.wheel_steering) != self._state.wheel_count:
            self._cmd.wheel_steering = [0.0]*self._state.wheel_count

        if len(self._cmd.wheel_power) != self._state.wheel_count:
            self._cmd.wheel_power = [0.0] * self._state.wheel_count

        if len(self._cmd.wheel_brake) != self._state.wheel_count:
            self._cmd.wheel_brake = [0.0]*self._state.wheel_count

        self._initialized = True

    def set_powered_wheel_indices(self, indices):
        # Sanity check, make sure that none of the indexes are greater than the wheel count
        for idx in indices:
            if not self.check_wheel_count(idx):
                return False

        self._powered_wheel_indices = indices
        self.initialize_wheel_cmds()

    def set_vehicle_steering(self, val):
        """
        Use the user defined wheel settings to automatically set the steering of appropriate wheels
        :param val:
        :return:
        """
        if not self._steering_wheel_indices:
            print("Please set the steering wheel indices before calling this method")
        else:
            for idx in self._steering_wheel_indices:
                self._cmd.wheel_steering[idx] = val

    def set_vehicle_steering_ackerman(self, val):
        """
        Use the user defined wheel settings to automatically set the steering of appropriate wheels. Implement Ackerman
        steering.
        :param val:
        :return:
        """
        if not self._steering_wheel_indices:
            print("Please set the steering wheel indices before calling this method")
        else:
            for idx in self._steering_wheel_indices:
                self._cmd.wheel_steering[idx] = val

    def set_vehicle_power(self, val):
        """
        Use the user defined wheel settings to automatically set the power of appropriate wheels
        :param val:
        :return:
        """
        if not self._powered_wheel_indices:
            print("Please set the powered wheel indices before calling this method")
        else:
            for idx in self._powered_wheel_indices:
                self._cmd.wheel_power[idx] = val

    def set_vehicle_brake(self, val):
        """
        Use the user defined wheel settings to automatically set the power of appropriate wheels
        :param val:
        :return:
        """
        if not self._powered_wheel_indices:
            print("Please set the powered wheel indices before calling this method")
        else:
            for idx in self._powered_wheel_indices:
                self._cmd.wheel_brake[idx] = val

    def set_wheel_steering(self, idx, val):
        """
        Manually define the wheel which to steer. The user is responsible for knowing what the steering limits are
        :param idx:
        :param val:
        :return:
        """
        if not self._initialized:
            self.initialize_wheel_cmds()

        if self.check_wheel_count(idx):
            self._cmd.wheel_steering[idx] = val

    def set_wheel_power(self, idx, val):
        """
        Manually define the wheel which to power. The user is responsible for knowing what the engine power limits are
        :param idx:
        :param val:
        :return:
        """
        if not self._initialized:
            self.initialize_wheel_cmds()

        if self.check_wheel_count(idx):
            self._cmd.wheel_power[idx] = val

    def set_wheel_brake(self, idx, val):
        """
        Manually define the wheel which to brake. The user is responsible for knowing what the engine brake limits are
        :param idx:
        :param val:
        :return:
        """
        if not self._initialized:
            self.initialize_wheel_cmds()

        if self.check_wheel_count(idx):
            self._cmd.wheel_brake[idx] = val
