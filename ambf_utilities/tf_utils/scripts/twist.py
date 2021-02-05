""" This class will generate a Vector in 3D to mimic Vector in pyKDL"""

import numpy as np
from vector import Vector


class Twist(object):
  rot = Vector()
  vel = Vector()

  def __init__(self, rot=None, vel=None):

    super(Twist, self).__init__()
    if rot is not None:
      self.rot = rot

    if vel is not None:
      self.vel = vel

  def RefPoint(self):
    """
        Changes the reference point of the Twist. The vector v_base_AB is expressed 
        in the same base as the twist The vector v_base_AB is a vector from the old 
        point to the new point.

        ***NOT IMPLEMENTED YET***
        """
    return

  def ReverseSign(self):
    """ 
            Reverse the sign of rot and vel components

            @return None
        """

    self.rot = np.negative(self.rot)
    self.vel = np.negative(self.vel)

    return

  def Zero(self):
    """ 
            Returns a zero Twist

            @return Twist
        """

    self.rot = Vector()
    self.vel = Vector()
    return self

  def __add__(self, t2):
    return Twist.add(self, t2)

  def __sub__(self, t2):
    return Twist.sub(self, t2)

  def __repr__(self):
    return type(self)

  def __str__(self):
    return "rot = {}\nvel = {}".format(self.rot, self.vel)

  @staticmethod
  def add(t1, t2):
    w = Twist()

    w.rot = t1.rot + t2.rot
    w.vel = t1.vel + t2.vel

    return w

  @staticmethod
  def sub(t1, t2):
    w = Twist()

    w.rot = t1.rot - t2.rot
    w.vel = t1.vel - t2.vel

    return w


def test():
  t1 = Twist(rot=Vector(1, 2, -1), vel=Vector(2, 1, -2))
  t2 = Twist(rot=Vector(0, 1, -1), vel=Vector(2, 2, 0))
  print(t1)
  print(t2)
  print(t1 - t2)
  t1 += t2
  print(t1)


if __name__ == '__main__':
  test()
