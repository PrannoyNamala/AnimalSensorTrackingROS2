from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'floating_object'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prannoy',
    maintainer_email='prannoy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radio_signal_action_server = floating_object.radio_signal_action_server:main',
            'radio_signal_action_client = floating_object.radio_signal_action_client:main',
            'spawn_floating_object = floating_object.spawn_floating_object:main',
            'position_publisher = floating_object.position_publisher:main',
        ],
    },
)
