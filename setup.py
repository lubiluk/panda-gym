# -*- coding: utf-8 -*-
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

with open('README.md', 'r') as f:
    long_description = f.read()

setup_args = generate_distutils_setup(
    packages=['panda-gym'],
    package_dir={'': 'src'},
    long_description=long_description,
    long_description_content_type='text/markdown',
)

setup(**setup_args)