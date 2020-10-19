#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['metrics_refbox_client', 'metrics_benchmark_mockup'],
    package_dir={'metrics_refbox_client': 'src/metrics_refbox_client',
                 'metrics_benchmark_mockup': 'src/metrics_benchmark_mockup'}
)

setup(**d)
