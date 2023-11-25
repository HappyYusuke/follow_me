from setuptools import find_packages, setup

package_name = 'recognition_by_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'detect_laser_img = recognition_by_lidar.detect_laser_img:main',
            'laser_imgfile_maker = recognition_by_lidar.laser_imgfile_maker:main',
            'base_controller = recognition_by_lidar.base_controller:main',
        ],
    },
)
