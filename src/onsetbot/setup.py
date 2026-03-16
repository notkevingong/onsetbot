from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'onsetbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gong',
    maintainer_email='gong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'actuator_command = onsetbot.actuator_command:main',
            'odrive_can_bridge = onsetbot.odrive_can_bridge:main',
            'onset_gui = onsetbot.onset_gui:main',
            'stm32_bridge = onsetbot.stm32_bridge:main',
            'actuator_commands = onsetbot.actuator_commands:main',
        ],
    },
)
