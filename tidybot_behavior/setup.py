from setuptools import setup
import os
from glob import glob

package_name = 'tidybot_behavior'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skandy',
    maintainer_email='skandy@todo.com',
    description='Behavior nodes for the TidyBot home-tidying robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_manager     = tidybot_behavior.task_manager:main',
            'object_detector  = tidybot_behavior.object_detector:main',
            'arm_controller   = tidybot_behavior.arm_controller:main',
            'pick_and_place   = tidybot_behavior.pick_and_place:main',
            'collision_monitor = tidybot_behavior.collision_monitor:main',
        ],
    },
)
