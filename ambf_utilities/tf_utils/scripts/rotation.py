""" This class will generate a Rotation in 3D to mimic Rotation in pyKDL"""

import numpy as np
from numpy import pi, sin, cos, tan, arctan2
from vector import Vector
from twist import Twist
from wrench import Wrench
from warnings import warn
import sys

class Rotation(np.ndarray):
    
    X_AX = Vector(1., 0., 0.)
    Y_AX = Vector(0., 1., 0.)
    Z_AX = Vector(0., 0., 1.)

    epsilon = 1e-12
    
    @staticmethod
    def RPY(roll=0., pitch=0., yaw=0.):
        """
        Get the rotation matrix for a rpy

        @param float roll
        @param float pitch
        @param float yaw

        @return Rotation
        """

        ca1 = cos(yaw)
        sa1 = sin(yaw)
        cb1 = cos(pitch)
        sb1 = sin(pitch)
        cc1 = cos(roll)
        sc1 = sin(roll)
        
        return Rotation(ca1*cb1, ca1*sb1*sc1 - sa1*cc1, ca1*sb1*cc1 + sa1*sc1,
                   sa1*cb1, sa1*sb1*sc1 + ca1*cc1, sa1*sb1*cc1 - ca1*sc1,
                   -sb1, cb1*sc1, cb1*cc1)
        
    @staticmethod
    def EulerZYZ(z1=0., y=0., z2=0.):
        """
        Get the rotation matrix for a ZYZ rotation

        @param float z1
        @param float y
        @param float z2

        @return Rotation
        """

        sa  = sin(z1)
        ca = cos(z1)
        sb  = sin(y)
        cb = cos(y)
        sg  = sin(z2)
        cg = cos(z2)
        
        return Rotation(ca*cb*cg-sa*sg, -ca*cb*sg-sa*cg, ca*sb,
                 sa*cb*cg+ca*sg, -sa*cb*sg+ca*cg, sa*sb,
                 -sb*cg, sb*sg, cb
                )

    @staticmethod
    def EulerZYX(x=0., y=0., z=0.):
        """
        Get the rotation matrix for a ZYX rotation

        @param float z
        @param float y
        @param float x

        @return Rotation
        """
        
        return Rotation.RPY(z, y, x)

    @staticmethod
    def is_valid(mat):
        """ 
        Checks if the rotation is a valid rotation matrix. Also checks for 
        singularities

        @param      mat 3x3
        @returns    mat
        """
        x = Vector(mat[:,0])
        y = Vector(mat[:,1])
        z = Vector(mat[:,2])
        try:
            vNorms = Vector(x.Norm(), y.Norm(), z.Norm())
            assert not np.any(vNorms[vNorms > 1])
            assert not np.any(mat[mat > 1])
        except AssertionError as norm_error:
            warn("Values are not normalized. Auto normalize will take place.")
            x.Normalize()
            y.Normalize()
            z.Normalize()

        try:
            assert not np.array_equal(x,y)
            assert not np.array_equal(x,z)
            assert not np.array_equal(z,y)
            x = Vector(mat[0,:])
            y = Vector(mat[1,:])
            z = Vector(mat[2,:])
            assert not np.array_equal(x,y)
            assert not np.array_equal(x,z)
            assert not np.array_equal(z,y)
        except AssertionError as singular_error:
            warn("Rotation is singular, raising exception to exit")
            sys.exit()
        
        mat[:,0] = x
        mat[:,1] = y
        mat[:,2] = z

        return mat

    @staticmethod
    def Rot(axis, angle):
        """
        Get the rotation matrix using the axis and angle

        @param Vector   axis
        @param float    angle

        @return Rotation
        """
        axis.Normalize()

        ct = cos(angle)
        st = sin(angle)
        vt = 1-ct
        m_vt_0=vt*axis[0]
        m_vt_1=vt*axis[1]
        m_vt_2=vt*axis[2]
        m_st_0=axis[0]*st
        m_st_1=axis[1]*st
        m_st_2=axis[2]*st
        m_vt_0_1=m_vt_0*axis[1]
        m_vt_0_2=m_vt_0*axis[2]
        m_vt_1_2=m_vt_1*axis[2]

        return Rotation(
            ct      +  m_vt_0*axis[0],
            -m_st_2 +  m_vt_0_1,
            m_st_1  +  m_vt_0_2,
            m_st_2  +  m_vt_0_1,
            ct      +  m_vt_1*axis[1],
            -m_st_0 +  m_vt_1_2,
            -m_st_1 +  m_vt_0_2,
            m_st_0  +  m_vt_1_2,
            ct      +  m_vt_2*axis[2]
            )

    @staticmethod
    def Identity():
        """
        Get the Identity rotation matrix

        @return Rotation
        """

        return Rotation()

    @staticmethod
    def Quaternion(x, y, z, w):
        """
        Get the rotation matrix for a quaternion rotation

        @param float x
        @param float y
        @param float z
        @param float w

        @return Rotation
        """

        x2 = x*x
        y2 = y*y
        z2 = z*z
        w2 = w*w

        return Rotation(w2+x2-y2-z2, 2*x*y-2*w*z, 2*x*z+2*w*y,
                        2*x*y+2*w*z, w2-x2+y2-z2, 2*y*z-2*w*x,
                        2*x*z-2*w*y, 2*y*z+2*w*x, w2-x2-y2+z2)

    def __new__(cls, *args):
        obj = None
        if len(args) == 3:
            # x, y, z vectors
            mat = np.identity(3)
            for i in range(3):
                for j in range(3):
                    mat[j,i] = args[i][j]
            mat = Rotation.is_valid(mat)
            obj = np.asarray(mat).view(cls)
            obj.info = "Rotation of type dir_vectors"
        elif len(args) == 4:
            # x, y, z vectors
            mat = np.identity(3)
            for i in range(3):
                for j in range(3):
                    mat[j,i] = args[i][j]
            mat = Rotation.is_valid(mat)
            obj = np.asarray(mat).view(cls)
            obj.info = args[3]
        elif len(args) == 9:
            # Don't know whether pyKDL does it row or column vector based
            # Need to circle back during testing
            # Doing it the Col vector way for now.
            # Xx, Yx, Zx, Xy, Yy, Zy, Xz, Yz, Zz
            mat = np.identity(3).flatten()
            x = None
            y = None
            z = None
            for i in range(9):
                mat[i] = args[i]
            mat = mat.reshape(3,3)
            mat = Rotation.is_valid(mat)
            obj = np.asarray(mat).view(cls)
            obj.info = "Rotation of type ele"
        elif len(args) == 10:
            # Xx, Yx, Zx, Xy, Yy, Zy, Xz, Yz, Zz, info
            mat = np.identity(3).flatten()
            x = None
            y = None
            z = None
            for i in range(9):
                mat[i] = args[i]
            mat = mat.reshape(3,3)
            mat = Rotation.is_valid(mat)
            obj = np.asarray(mat).view(cls)
            obj.info = args[9]
        elif len(args) == 1:
            mat = Rotation.is_valid(args[0])
            obj = np.asarray(mat).view(cls)
            obj.info = "Rotation of type Rotation"
        else:
            # identity
            mat = np.identity(3)
            obj = np.asarray(mat).view(cls)
            obj.info = "Rotation of type Identity"
            
        return obj

    def __array_finalize__(self, obj):
        # see InfoArray.__array_finalize__ for comments
        if obj is None: return
        self.info = getattr(obj, 'info', None)
    
    def DoRotX(self, angle, unit='rad'):
        """
        Rotate about the x-axis with angle in radians

        @param angle in radians (default)
        @param unit can be 'rad' or 'deg'

        @return None
        """

        if unit == 'deg':
            angle = float(angle) * (pi / 180)
        
        self = Rotation.Rot(self.X_AX, angle)

    def DoRotY(self, angle, unit='rad'):
        """
        Rotate about the y-axis with angle in radians

        @param angle in radians (default)
        @param unit can be 'rad' or 'deg'

        @return None
        """
        if unit == 'deg':
            angle = float(angle) * (pi / 180)
        
        self = Rotation.Rot(self.Y_AX, angle)

    def DoRotZ(self, angle, unit='rad'):
        """
        Rotate about the z-axis with angle in radians

        @param angle in radians (default)
        @param unit can be 'rad' or 'deg'

        @return None
        """
        if unit == 'deg':
            angle = float(angle) * (pi / 180)
        
        self = Rotation.Rot(self.Z_AX, angle)
    
    def GetEulerZYX(self):
        """
        EulerZYX constructs a Rotation from the Euler ZYX parameters:

        First rotate around Z with z1,
        then around the new Y with y,
        then around new X with z2.
        Closely related to RPY-convention.

        Invariants:

        EulerZYX(alpha,y,z2) == EulerZYX(alpha +/- PI, PI-y, z2 +/- PI)
        (angle + 2*k*PI)

        @return (z, y, x)
        """

        return self.GetRPY()

    def GetEulerZYZ(self):
        """
        Returns the euler angles for rotations about the Z1, Y and Z2 axes

        @return (z1, y, z2)
        """

        alpha=0.0
        beta=0.0
        gamma=0.0
        if np.abs(self[2,1]) > 1-epsilon:
            gamma=0.0
            if self[2,1] > 0:
                beta = 0.0
                alpha = arctan2(self[1,0], self[0,0])
            else:
                beta = pi
                alpha = arctan2(-self[1,0], -self[0,0])
        else:
            alpha = arctan2(self[1,2], self[0,2])
            beta = arctan2(np.sqrt(self[2,0]**2 + self[2,1]**2), self[2,2])
            gamma = arctan2(self[2,1], -self[2,0])

        return (alpha, beta, gamma)

    def GetQuaterninon(self):
        """
        Returns the normalized quaternion that describes the rotation

        @return (x, y, z, w)
        """

        trace = self[0,0] + self[1,1] + self[2,2]

        x = 0.
        y = 0.
        z = 0.
        w = 0.
        
        if trace > self.epsilon:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = ( self[2,1] - self[1,2] ) * s
            y = ( self[0,2] - self[2,0] ) * s
            z = ( self[1,0] - self[0,1] ) * s
        else:
            if self[0,0] > self[1,1] and self[0,0] > self[2,2]:
                s = 2.0 * np.sqrt( 1.0 + self[0,0] - self[1,1] - self[2,2])
                w = (self[2,1] - self[1,2] ) / s
                x = 0.25 * s
                y = (self[0,1] + self[1,0] ) / s
                z = (self[0,2] + self[2,0] ) / s
            elif self[1,1] > self[2,2]:
                s = 2.0 * np.sqrt( 1.0 + self[1,1] - self[0,0] - self[2,2])
                w = (self[0,2] - self[2,0] ) / s
                x = (self[0,1] + self[1,0] ) / s
                y = 0.25 * s
                z = (self[1,2] + self[2,1] ) / s
            else:
                s = 2.0 * np.sqrt( 1.0 + self[2,2] - self[0,0] - self[1,1] )
                w = (self[1,0] - self[0,1] ) / s
                x = (self[0,2] + self[2,0] ) / s
                y = (self[1,2] + self[2,1] ) / s
                z = 0.25 * s

        return (x, y, z, w)

    def GetRPY(self):
        """
        Returns the r, p, y rotations around fixed axis that describe this rotation. 
        First a rotation around the x-axis, then a rotation around the original 
        y-axis, and finally a rotation around the original z-axis

        @return (r, p, y)
        """
        
        roll = None
        yaw = None
        pitch = arctan2(self[2,0], np.sqrt(self[0,0]**2 + self[1,0]**2))
        if np.abs(pitch) > (pi/2.) - self.epsilon:
            yaw = arctan2(-self[0,1], self[2,2])
            roll = 0.
        else:
            roll = arctan2(self[2,1], self[2,2])
            yaw = arctan2(self[1,0], self[0,0])
        
        return (roll, pitch, yaw)

    def GetRot(self):
        """
        Returns a vector with the direction of the equivalent axis and its norm the angle. 
        This method returns the axis as a Vector

        @return axis Vector
        """

        (axis, angle) = self.GetRotAngle()
        return Vector(axis*angle)

    def GetRotAngle(self):
        """
        Returns the rotation angle around the equivalent axis. This method returns the 
        angle as a double, and the rotation axis as a Vector

        @return (angle, axis)
        """

        angle = 0.
        x = 0.
        y = 0.
        z = 0.
        
        epsilon = self.epsilon
        epsilon2 = self.epsilon*10 # margin to distinguish between 0 and 180 degrees
        # optional check that input is pure rotation, 'isRotationMatrix' is defined at:
        # http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/
        # Can use is_valid instead?
        if ((np.abs(self[0,1] - self[1,0]) < epsilon)
            and (np.abs(self[0,2] - self[2,0]) < epsilon)
            and (np.abs(self[1,2] - self[2,1]) < epsilon)):
            # singularity found
            # first check for identity matrix which must have +1 for all terms
            # in leading diagonal and zero in other terms
            if ((np.abs(self[0,1] + self[1,0]) < epsilon2)
                and (np.abs(self[0,2] + self[2,0]) < epsilon2)
                and (np.abs(self[1,2] + self[2,1]) < epsilon2)
                and (np.abs(self[0,0] + self[1,1] + self[2,2]-3) < epsilon2)):
                # this singularity is identity matrix so angle = 0, axis is arbitrary
                # Choose 0, 0, 1 to pass orocos tests
                axis = Vector(0,0,1)
                angle = 0.0
                return (axis, angle)
            
            # otherwise this singularity is angle = 180
            angle = pi
            xx = (self[0,0] + 1) / 2
            yy = (self[1,1] + 1) / 2
            zz = (self[2,2] + 1) / 2 
            xy = (self[0,1] + self[1,0]) / 4
            xz = (self[0,2] + self[2,0]) / 4
            yz = (self[1,2] + self[2,1]) / 4
            
            if ((xx > yy) and (xx > zz)):
                # self[0,0] is the largest diagonal term
                x = np.sqrt(xx)
                y = xy/x
                z = xz/x
            
            elif (yy > zz):
                # self[1,1] is the largest diagonal term
                y = np.sqrt(yy)
                x = xy/y
                z = yz/y
            
            else:
                # self[2,2] is the largest diagonal term so base result on this
                x = xz/z
                y = yz/z
            
            axis = Vector(x, y, z)
            return (axis, angle) # return 180 deg rotation

        # If the matrix is slightly non-orthogonal, `f` may be out of the (-1, +1) range.
        # Therefore, clamp it between those values to avoid NaNs.
        f = (self[0,0] + self[1,1] + self[2,2] - 1) / 2
        angle = np.arccos(np.max(-1.0, np.min(1.0, f)))

        x = (self[2,1] - self[1,2])
        y = (self[0,2] - self[2,0])
        z = (self[1,0] - self[0,1])
        axis = Vector(x, y, z)
        axis.Normalize()
        return (axis, angle)

    def Inverse(self):
        """
        Returns the inverse rotation (this is also the transpose of the rotation matrix)

        @return Rotation
        """

        return Rotation(np.transpose(self))
    
    def SetInverse(self):
        """
        Sets the inverse rotation (this is also the transpose of the rotation matrix)

        @return None
        """

        self = self.Inverse()

        return
    
    def UnitX(self):
        """
        Returns the column vector for x

        @return Vector
        """

        return Vector(self[0,0], self[1,0], self[2,0])
    
    def UnitY(self):
        """
        Returns the column vector for x

        @return Vector
        """
        
        return Vector(self[0,1], self[1,1], self[2,1])
    
    def UnitZ(self):
        """
        Returns the column vector for x

        @return Vector
        """
        
        return Vector(self[0,2], self[1,2], self[2,2])
    
    def __mul__(self, arg):

        try:
            if isinstance(arg, Vector):
                """
                Changes the reference frame of a Vector. The norm of the vector does not change.
                """
                return Vector(arg.dot(self))
                
            elif isinstance(arg, Twist):
                """
                Changes the reference frame of a Twist
                """

                return Twist(arg.rot*self, arg.vel)

            elif isinstance(arg, Wrench):
                """
                Changes the reference frame of a Wrench
                """
                
                return Wrench(arg.force*self, arg.torque*self)
            
            elif isinstance(arg, Rotation):
                """
                Changes the reference frame of a Wrench
                """
                
                return Rotation(np.matmul(self, arg))

            else:
                raise TypeError("arg is not supported for this multiplication")
        except TypeError as error:
            sys.exit()
        
        return None

def vec_test():
    Vec1 = Vector(1,2,-1)
    Vec2 = Vector(2,1,-2)
    Vec1.Normalize()
    print (Vec1)
    print (Vec2)
    print (Vec1.dot(Vec2))
    print (Vec1.dot(Vec2))
    print (Vec1 * Vec2)
    Vec1 = Vector(Vec2)
    print (Vec1)
    Vec1 += Vec2
    print (Vec1)

def test():
    rot = Rotation()
    X = rot.Rot(Rotation.X_AX,pi/4)
    Y = rot.Rot(Rotation.Y_AX,pi/3)
    Z = rot.Rot(Rotation.Z_AX,pi/6)
    rot = X*Y*Z
    print(rot)
    (x, y, z, w) = rot.GetQuaterninon()
    print(x, y, z, w)
    rot = Rotation.Quaternion(x, y, z, w)
    (roll, pitch, yaw) = rot.GetRPY()
    rot = Rotation.RPY(roll, pitch, yaw)
    (x, y, z, w) = rot.GetQuaterninon()
    print(x, y, z, w)

if __name__ == '__main__':
    test()
