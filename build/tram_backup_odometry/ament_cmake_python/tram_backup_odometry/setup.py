from setuptools import find_packages
from setuptools import setup

setup(
    name='tram_backup_odometry',
    version='1.0.0',
    packages=find_packages(
        include=('tram_backup_odometry', 'tram_backup_odometry.*')),
)
