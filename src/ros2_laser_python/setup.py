from setuptools import setup

package_name = 'ros2_laser_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akoubaa',
    maintainer_email='anis.koubaa@gmail.com',
    description='This ROS2 Python package deals with learning Laser Scanners',
    license='Creative Commons Non-Commercial (CC-NC)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_info= ros2_laser_python.scan_info:main',
            'scan_info_thread= ros2_laser_python.scan_info_thread:main',
            'scan_info_thread_v2= ros2_laser_python.scan_info_thread_v2:main',
            'scan_info_asyncio= ros2_laser_python.scan_info_asyncio:main',
            'scan_info_executors= ros2_laser_python.scan_info_executors:main',
            'move_stop_obstacle= ros2_laser_python.move_stop_obstacle:main',
        ],
    },
)
