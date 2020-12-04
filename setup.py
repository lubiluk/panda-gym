# -*- coding: utf-8 -*-
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# from setuptools import setup
# from catkin_pkg.python_setup import generate_distutils_setup

# with open('README.md', 'r') as f:
#     long_description = f.read()

# setup_args = generate_distutils_setup(
#     packages=['panda-gym'],
#     package_dir={'': 'src'},
#     long_description=long_description,
#     long_description_content_type='text/markdown',
# )

# setup(**setup_args)

from setuptools import setup, find_packages

with open('README.md', 'r') as f:
    long_description = f.read()

setup(
    name='panda_gym',
    description='OpenAI Gym Franka Emika Panda robot environment based on Gazebo.',
    author='Quentin GALLOUÉDEC',
    author_email='gallouedec.quentin@gmail.com',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/quenting44/panda-gym',
    packages=find_packages(),
    include_package_data=True,
    package_data={
        'panda_gym': ['envs/assets/*.json']
    },
    version='0.1.0',
    install_requires=['gym', 'pybullet', 'numpy']
)