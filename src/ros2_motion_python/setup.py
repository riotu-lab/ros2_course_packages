from setuptools import setup

package_name = 'ros2_motion_python'

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
    maintainer='Anis Koubaa',
    maintainer_email='anis.koubaa@gmail.com',
    description='Learn about ROS2 motion',
    license='GNU',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mover= ros2_motion_python.simple_turtlesim_motion:main',
            'cleaner= ros2_motion_python.clean:main',
        ],
    },
)
