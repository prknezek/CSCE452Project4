from setuptools import find_packages, setup

package_name = 'project4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('lib/project4', ['project4/particle_filter_localization.py']),
        ('lib/project4', ['project4/occupancy_grid_publisher.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grant',
    maintainer_email='gshields432@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_grid_publisher = project4.occupancy_grid_publisher:main',
            'particle_filter_localization = project4.particle_filter_localization:main',
        ],
    },
)
