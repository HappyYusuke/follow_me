import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'recognition_by_lidar'
modules = 'recognition_by_lidar/modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, modules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yusuke-wsl',
    maintainer_email='yusuke-wsl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_to_img = recognition_by_lidar.laser_to_img:main',
            'laser_imgshow = recognition_by_lidar.laser_imgshow:main',
            'person_detector = recognition_by_lidar.person_detector :main',
            'laser_imgfile_maker = recognition_by_lidar.laser_imgfile_maker:main',
            'base_controller = recognition_by_lidar.base_controller:main',
        ],
    },
)
