""" This class will generate a Frame in 3D space to mimic Frame in pyKDL"""

import numpy as np
from numpy import pi, sin, cos, tan, arctan2
from numpy.core.numeric import identity
from vector import Vector
from twist import Twist
from wrench import Wrench
from rotation import Rotation
from warnings import warn
import sys


class Frame(object):

  @staticmethod
  def make_Frame(HTM):
    """
    Return a frame from a 4x4 Homogenous transformaiton matrix

    @param 4x4mat Homogenous TF Matrix

    @return Frame
    """

    frame = Frame(HTM[:3, :3], HTM[0:3, 3])
    return frame

  @staticmethod
  def make_HTM(f):
    """
    Return a 4x4 Homogenous Transformation Matrix from a Frame

    @param Frame f
    
    @return 4x4mat Homogenous TF Matrix
    """
    
    mat_44 = np.ndarray((4, 4))
    mat_44[:3, :3] = f.M
    mat_44[3:, :4] = np.array([0, 0, 0, 1])
    mat_44[:3, 3] = f.p

    return mat_44

  @staticmethod
  def Identity():
    """
        Constructs an identity frame

        @return Frame
        """

    return Frame()

  @staticmethod
  def DH(a, alpha, d, theta):
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
    return Frame(
      Rotation(ct,
               -st * ca,
               st * sa,
               st,
               ct * ca,
               -ct * sa,
               0,
               sa,
               ca),
      Vector(a * ct,
             a * st,
             d)
    )

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
    return Frame(
      Rotation(ct,
               -st,
               0,
               st * ca,
               ct * ca,
               -sa,
               st * sa,
               ct * sa,
               ca),
      Vector(a,
             -sa * d,
             ca * d)
    )

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

    self.M = None
    self.p = None

    if rot is not None:
      self.M = Rotation(rot)
    else:
      self.M = Rotation()

    if pos is not None:
      self.p = Vector(pos)
    else:
      self.p = Vector()

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
    self.p = -self.M * self.p

    return self

  def __mul__(self, f):

    this_mat_44 = Frame.make_HTM(self)
    f_mat_44 = Frame.make_HTM(f)

    mul = np.matmul(this_mat_44, f_mat_44)

    frame = Frame.make_Frame(mul)

    return frame


def test():
  f1 = Frame()
  f2 = Frame()
  f = f1 * f2
  print(f.p)
  print(f.M)


if __name__ == '__main__':
  test()
