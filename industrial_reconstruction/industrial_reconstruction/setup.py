from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['industrial_reconstruction', 'industrial_reconstruction.utility'],
    package_dir={'': 'src'},
)

setup(**d)
