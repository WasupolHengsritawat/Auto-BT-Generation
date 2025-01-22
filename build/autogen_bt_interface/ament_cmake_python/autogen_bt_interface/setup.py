from setuptools import find_packages
from setuptools import setup

setup(
    name='autogen_bt_interface',
    version='0.0.0',
    packages=find_packages(
        include=('autogen_bt_interface', 'autogen_bt_interface.*')),
)
