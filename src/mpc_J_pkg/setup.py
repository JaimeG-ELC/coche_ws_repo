from setuptools import setup

package_name = 'mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob.glob('launch/*.py')),  # Asegura que se copian todos los scripts
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JayCeeON',
    maintainer_email='joaquincc1254@gmail.com',
    description='f1tenth mpc',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_node = mpc.mpc_node:main',
        ],
    },
)
