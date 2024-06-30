#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

from distutils.extension import Extension
from Cython.Build import cythonize


d = generate_distutils_setup(
    scripts=['bin/pera_gui'],
    packages=['pera_control'],
    package_dir={'': 'src'}
)

setup(**d)



