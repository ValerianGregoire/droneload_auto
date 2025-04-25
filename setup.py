from setuptools import find_packages, setup

package_name = 'droneload_auto'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valerian',
    maintainer_email='valerian.greg@gmail.com',
    description='ROS 2 package for the automation of the drone for the Safran Droneload challenge.',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
