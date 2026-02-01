from setuptools import find_packages, setup

package_name = 'odrive_tools'

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
            'velocity_commander = odrive_tools.vel_commander:main',
            'zero_sequence = odrive_tools.zero_sequence:main',
            'odrive_can_bridge = odrive_tools.odrive_can_bridge:main',
        ],
    },
)
