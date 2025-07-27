from setuptools import setup
from glob import glob
import os

package_name = 'f1tenth_stack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaime',
    maintainer_email='your.email@example.com',
    description='F1TENTH stack with test servo node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_servo = f1tenth_stack.test_servo:main',
        ],
    },
)
