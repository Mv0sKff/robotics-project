import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robotik_projekt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zion',
    maintainer_email='zion@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_place_node = robotik_projekt.pick_place_node:main',
            'pick_place_node_demo = robotik_projekt.pick_place_node_demo:main',
            'pick_place_moveit = robotik_projekt.pick_place_moveit:main',
            'pick_place_pymoveit2 = robotik_projekt.pick_place_pymoveit2:main',
            'pick_place_servo = robotik_projekt.pick_place_servo:main',
        ],
    },
)
