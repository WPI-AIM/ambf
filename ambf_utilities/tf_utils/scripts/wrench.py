""" This class will generate a Vector in 3D to mimic Vector in pyKDL"""

import numpy as np
from vector import Vector


class Wrench(object):
  force = Vector()
  torque = Vector()

  def __init__(self, force=None, torque=None):
    super(Wrench, self).__init__()
    if force is not None:
      self.force = force

    if torque is not None:
      self.torque = torque

  def RefPoint(self):
    """
        Changes the reference point of the wrench. The vector v_base_AB is expressed 
        in the same base as the twist The vector v_base_AB is a vector from the old 
        point to the new point.

        ***NOT IMPLEMENTED YET***
        """
    return

  def ReverseSign(self):
    """ 
            Reverse the sign of force and torque components

            @return None
        """

    self.force = np.negative(self.force)
    self.torque = np.negative(self.torque)

    return

  def Zero(self):
    """ 
            Returns a zero Wrench

            @return Wrench
        """

    self.force = Vector()
    self.torque = Vector()
    return self

  def __add__(self, w2):
    return Wrench.add(self, w2)

  def __sub__(self, w2):
    return Wrench.sub(self, w2)

  def __repr__(self):
    return type(self)

  def __str__(self):
    return "force = {}\ntorque = {}".format(self.force, self.torque)

  @staticmethod
  def add(w1, w2):
    w = Wrench()

    w.force = w1.force + w2.force
    w.torque = w1.torque + w2.torque

    return w

  @staticmethod
  def sub(w1, w2):
    w = Wrench()

    w.force = w1.force - w2.force
    w.torque = w1.torque - w2.torque

    return w


def test():
  W1 = Wrench(force=Vector(1, 2, -1), torque=Vector(2, 1, -2))
  W2 = Wrench(force=Vector(0, 1, -1), torque=Vector(2, 2, 0))
  print(W1)
  print(W2)
  print(W1 - W2)
  W1 += W2
  print(W1)


if __name__ == '__main__':
  test()
