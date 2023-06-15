import os
from glob import glob
from setuptools import setup

package_name = 'robu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + '/rviz', ['rviz/tb3_cartographer.rviz']),
        ('share/' + package_name + '/config', ['config/turtlebot3_lds_2d.lua']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LI',
    maintainer_email='li@htl-kaindorf.at',
    description='ROBU-Robotic Exercises',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hellorobu = robu.ex01_hellorobu:main',
            'remotectrl = robu.ex02_remotectrl:main',
            'remotectrl_listener = robu.ex02_remotectrl_listener:main',
            'oas = robu.ex03_obstacle_avoidance_simple:main',
            'camera = robu.ex07_camera_pub:main',
            'wiimotectrl = robu.wiimotectrl:main'
        ],
    },
)
