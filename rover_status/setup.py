from setuptools import find_packages, setup

package_name = 'rover_status'

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
    maintainer_email='tejask@todo.com',
    description='Notify battery level and health',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["battery_temp_node = rover_status.battery_temp_pub:main",   
        "battery_health_node = rover_status.battery_health:main",
        "battery_sub_node = rover_status.battery_sub:main",    ],
    },
)
