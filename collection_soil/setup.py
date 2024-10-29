from setuptools import find_packages, setup

package_name = 'collection_soil'

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
    description='Rover Soil Collection Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["collection_service = collection_soil.collection_service:main",
        "collection_client = collection_soil.collection_client:main",
        "rover_collection = collection_soil.rover_collection:main",
        ],
    },
)
