from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

extensions = [Extension("client", ["client.pyx"]),
	          Extension("object", ["object.pyx"]),
              Extension("world" , ["world.pyx"]) ,
              Extension("wd"  ,   ["wd.pyx"])  ,
              Extension("env"  ,  ["env.pyx"])   ]
setup(ext_modules = cythonize(extensions))

