import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'base_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.lua'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunwei',
    maintainer_email='sunweijm.sun@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_receiver_node = base_driver.ir_receiver:main',
            'hm_base_node = base_driver.hm_base_driver:main',
            'params_dumper_server = base_driver.params_dumper_server:main',
        ],
    },
)
