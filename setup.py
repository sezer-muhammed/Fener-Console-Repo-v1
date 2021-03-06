from setuptools import setup
import os
from glob import glob

package_name = 'fener_console'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sezer',
    maintainer_email='konirobotics@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "sensor_vis = fener_console.bno055_lidar_vis:main",
        "driver_keyboard = fener_console.driver_keyboard:main",
        "webcam = fener_console.webcam_publisher:main",
        ],
    },
)
