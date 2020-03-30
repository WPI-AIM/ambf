""" This class will generate a Frame in 3D space to mimic Frame in pyKDL"""

import numpy as np
from numpy import pi, sin, cos, tan, arctan2
#from vector import Vector
#from twist import Twist
#from wrench import Wrench
#from rotation import Rotation

import os
import sys

if "./scripts/" not in sys.path:
  sys.path.append("./scripts/")

from vector import Vector
from frame import Frame
from rotation import Rotation
from twist import Twist
from wrench import Wrench

from warnings import warn
import sys

class Frame(object):
    M = Rotation()
    p = Vector()

    @staticmethod
    def Identity():
        """
        Constructs an identity frame

        @return Frame
        """

        return Frame()

    @staticmethod
    def HD(a, alpha, d, theta):
        """
        Constructs a transformationmatrix T_link(i-1)_link(i) with the Denavit-Hartenberg 
        convention as described in the original publictation: Denavit, J. and Hartenberg, 
        R. S., A kinematic notation for lower-pair mechanisms based on matrices, 
        ASME Journal of Applied Mechanics, 23:215-221, 1955.

        @param float a
        @param float alpha (radians)
        @param float d
        @param float theta (radians)

        @return Frame
        """

        ct = cos(theta)
        st = sin(theta)
        sa = sin(alpha)
        ca = cos(alpha)
        return Frame(Rotation(ct,    -st*ca,   st*sa,
                              st,     ct*ca,  -ct*sa,
                               0,        sa,      ca),
                     Vector(a*ct,      a*st,       d))
        
    @staticmethod
    def DH_Craig1989(a, alpha, d, theta):
        """
        Constructs a transformationmatrix T_link(i-1)_link(i) with the Denavit-Hartenberg 
        convention as described in the Craigs book: Craig, J. J.,Introduction to Robotics: 
        Mechanics and Control, Addison-Wesley, isbn:0-201-10326-5, 1986.

        @param float a
        @param float alpha (radians)
        @param float d
        @param float theta (radians)

        @return Frame
        """

        ct = cos(theta)
        st = sin(theta)
        sa = sin(alpha)
        ca = cos(alpha)
        return Frame(Rotation(ct,       -st,     0,
                              st*ca,  ct*ca,   -sa,
                               st*sa,  ct*sa,    ca),
                     Vector(a,      -sa*d,  ca*d))

    @staticmethod
    def AddDelta(f, t, d):
        """
        NOT IMPLEMENTED
        Constructs a frame that is obtained by: starting from frame f, apply twist t, during time d

        @param Frame f
        @param Twist t
        @param float d

        @return Frame
        """

        return

    def __init__(self, rot=None, pos=None):
        """
        Construct an an object of frame

        @param Rotation rot
        @param Vector pos
        """
        super(Frame, self).__init__()

        if rot is not None:
            self.M = rot
        
        if pos is not None:
            self.p = pos

        return
    
    def Integrate(self, twist, frequency):
        """
        This frame is integrated into an updated frame with sample frequence, using first order integration
        NOT IMPLEMENTED
        """

        return
    
    def Inverse(self):
        """
        Returns the inverse of the frame

        @return Frame
        """

        self.M.SetInverse()
        self.p = -self.M*self.p

        return self
