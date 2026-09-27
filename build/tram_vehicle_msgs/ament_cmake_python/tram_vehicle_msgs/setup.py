from setuptools import find_packages
from setuptools import setup

setup(
    name='tram_vehicle_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('tram_vehicle_msgs', 'tram_vehicle_msgs.*')),
)
