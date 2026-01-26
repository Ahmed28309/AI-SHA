from setuptools import find_packages
from setuptools import setup

setup(
    name='bno055_imu',
    version='1.0.0',
    packages=find_packages(
        include=('bno055_imu', 'bno055_imu.*')),
)
