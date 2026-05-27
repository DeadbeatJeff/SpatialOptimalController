from setuptools import find_packages, setup

package_name = 'safety_monitoring'

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
    maintainer='jeff',
    maintainer_email='DeadbeatJeffSDF@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = safety_monitoring.monitor_node:main',
            'mitigation = safety_monitoring.mitigation_node:main',
	    'spatial_control = safety_monitoring.spatial_optimal_controller:main',
        ],
    },
)
