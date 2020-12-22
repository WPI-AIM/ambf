""" This class will generate a Vector in 3D to mimic Vector in pyKDL"""

import numpy as np

class Vector(np.ndarray):

    def __new__(cls, *args):
        obj = None
        if len(args) == 1:
            # Vector
            input_array = args[0]
            obj = np.asarray(input_array).view(cls)
            obj.info = "Vector of type Vector/array"
        elif len(args) == 2:
            # Vector, info
            input_array = args[0]
            obj = np.asarray(input_array).view(cls)
            obj.info = "Vector of type Vector/array"
            obj.info = args[1]
        elif len(args) == 3:
            # x, y, z
            x = args[0]
            y = args[1]
            z = args[2]
            input_array = np.array([x, y, z])
            obj = np.asarray(input_array).view(cls)
            obj.info = "Vector of type x, y, z"
        elif len(args) == 4:
            # x, y, z, info
            x = args[0]
            y = args[1]
            z = args[2]
            input_array = np.array([x, y, z])
            obj = np.asarray(input_array).view(cls)
            obj.info = args[3]
        else:
            # zeros
            input_array = np.zeros((3,))
            obj = np.asarray(input_array).view(cls)
            obj.info = "Vector of Zeros"
            
        return obj

    def __array_finalize__(self, obj):
        # see InfoArray.__array_finalize__ for comments
        if obj is None: return
        self.info = getattr(obj, 'info', None)

    def Norm(self):
        """
        Return the norm of the vector

        @return float
        """

        return np.linalg.norm(self)
    
    def Normalize(self):
        """
        Normalize the vector in place, and return the norm of the vector

        @return float
        """

        self = np.divide(self, np.linalg.norm(self))
        self.info = "Normalized inplace"
        return np.linalg.norm(self)

    def ReverseSign(self):
        """ 
        Reverse the sign of the vector
        
        @return None
        """

        self = np.negative(self)
        return
    
    def Zero(self):
        """ 
        Sets all elements in the vector to zero
        
        @return None
        """
        self = np.zeros((1,3))
        return
    
    def x(self):
        """
        Return the x component of the vector

        @return float
        """

        return self[0]
    
    def y(self):
        """
        Return the y component of the vector

        @return float
        """

        return self[1]

    def z(self):
        """
        Return the z component of the vector

        @return float
        """

        return self[2]

    def __repr__(self):
        return type(self)
    
    def __str__(self):
        return "[{}, {}, {}]".format(self[0],self[1],self[2])

    def __mul__(self, v2):
        """
        Return the cross product with another vector

        @param Vector

        @return Vector
        """

        if isinstance(v2, Vector):
            return Vector.cross(self, v2)
        elif isinstance(v2, float):
            return np.multiply(v2, self)

    def __sub__(self, args):
        v2 = None
        output = None
        try:
            if len(args) == 1:
                # Vector
                input_array = args[0]
                if len(input_array) > 1:
                    v2 = Vector(input_array)
                else:
                    raise AssertionError
            else:
                v2 = Vector(args)

            output = np.subtract(self, v2)
        except AssertionError as error:
            sys.exit()

        return Vector(output)

    def __neg__(self):
        """ 
        Reverse the sign of the vector
        
        @return Vector
        """

        self.ReverseSign()
        return self
        
    
    def dot(self, v2):
        """
        Return the dot product with another vector

        @param Vector

        @return Vector
        """
        
        return Vector._dot(self, v2)
    
    @staticmethod
    def cross(v1, v2):
        v = np.cross(v1, v2)

        return Vector(v)
    
    @staticmethod
    def _dot(v1, v2):
        v = np.dot(v1[:], v2[:])

        return v

def dot(v1, v2):
    """
    Return the static method dot of the Vector class

    @param Vector
    @param Vector

    @return Vector
    """
    return Vector(v1).dot(Vector(v2))

def test():
    Vec1 = Vector(1,2,-1)
    Vec2 = Vector(2,1,-2)
    Vec1.Normalize()
    print ("Vector 1: ", Vec1)
    print ("Vector 2: ", Vec2)
    print ("V1.V2: ", Vec1.dot(Vec2))
    print ("V1 * V2: ", Vec1 * Vec2)
    Vec1 = Vector(Vec2)
    print ("Vector 1 = Vector 2: ", Vec1)
    Vec1 += Vec2
    print ("Vector 2 + Vector 2: ", Vec1)

if __name__ == '__main__':
    test()
