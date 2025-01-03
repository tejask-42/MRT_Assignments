from setuptools import find_packages, setup

package_name = 'rover_movement'

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
    maintainer='tejask',
    maintainer_email='tejask@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["obstacle_avoidance = rover_movement.obstacles:main",
        "rover_navigation = rover_movement.rover_nav:main",
        "map_pub = rover_movement.obstacle_map_pub:main",
        ],
    },
)
