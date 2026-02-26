from setuptools import find_packages
from setuptools import setup

setup(
    name='onset_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('onset_interfaces', 'onset_interfaces.*')),
)
