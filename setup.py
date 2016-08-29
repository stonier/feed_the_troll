#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['feed_the_troll'],
    package_dir={'': 'src'},
    # scripts=['scripts/demo_feeder',
    #          'scripts/demo_troll',
    #          ],
)
setup(**d)
