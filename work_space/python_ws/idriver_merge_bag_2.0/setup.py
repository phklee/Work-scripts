from distutils.core import setup
from Cython.Build import cythonize
setup(name='merge_bag',
     ext_modules=cythonize('merge_bag.py')) 