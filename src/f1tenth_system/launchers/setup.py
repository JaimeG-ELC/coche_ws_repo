from setuptools import setup, find_packages
import glob

package_name = 'launchers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Encuentra automáticamente los paquetes
    data_files=[
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob.glob('launch/*.py')),  # Asegura que se copian todos los scripts
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JayCeeON',
    maintainer_email='joaquincc1254@gmail.com',
    description='controllers launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.pure_pursuit_node:main',
        ],
    },
)

