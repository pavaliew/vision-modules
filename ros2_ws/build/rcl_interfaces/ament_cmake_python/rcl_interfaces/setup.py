from setuptools import find_packages
from setuptools import setup

setup(
    name='rcl_interfaces',
    version='2.4.2',
    packages=find_packages(
        include=('rcl_interfaces', 'rcl_interfaces.*')),
)
