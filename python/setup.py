#!/usr/bin/env python3

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
 
module1 = Extension("openhmd", 
    ["pyopenhmd.pyx", "pyopenhmd_wrapper.cpp"],
    language="c++",
    libraries=["openhmd"],
    library_dirs=['../'],
    include_dirs=['../include'])
 
setup(name = 'openhmd',
    version = '1.0',
    description = 'Python OpenHMD Wrapper',
    ext_modules=[module1],
    cmdclass = {'build_ext': build_ext})
