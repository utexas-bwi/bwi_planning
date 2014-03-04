#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bwi_planning_icaps14'],
    package_dir={'': 'src'},
    scripts={'scripts/planner_icaps2014'}
)

setup(**d)
