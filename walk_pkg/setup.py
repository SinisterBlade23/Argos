from setuptools import find_packages, setup

package_name = 'walk_pkg'

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
    maintainer='rushil',
    maintainer_email='kapoorrushil11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'forward_walk = walk_pkg.walk_forward_node:main',
            'idle = walk_pkg.default_position_node:main',
            'robot_control = walk_pkg.robot_control:main',
            'backward_walk = walk_pkg.walk_backward_node:main',
            'jump = walk_pkg.jump_node:main'
        ],
    },
)
