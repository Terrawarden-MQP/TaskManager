from setuptools import setup

package_name = 'joisie_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/all_nodes.launch.py']),
        ('share/' + package_name + '/launch', ['launch/just_manager.launch.py']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WPI Terrawarden Drone Cleanup MQP',
    maintainer_email='gr-dronecleanup@wpi.edu',
    description='ROS2 package for managing aerial manipulator code flow',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	        'task_manager = joisie_manager.taskmanager:main',
        ],
    },
)
