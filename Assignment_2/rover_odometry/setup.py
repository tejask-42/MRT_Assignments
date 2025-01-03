from setuptools import find_packages, setup

package_name = 'rover_odometry'

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
    maintainer='tejas',
    maintainer_email='tejaskulkarni785@gmail.com',
    description='Rover Odometry Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["odo_pub_node = rover_odometry.rover_odo_pub:main",
        "odo_sub_node = rover_odometry.rover_odo_sub:main",
        ],
    },
)
