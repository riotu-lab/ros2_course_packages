from setuptools import setup

package_name = 'ros2_essential_python'

setup(
    name=package_name,
    version='0.1.1',
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
    description='This packages provides essential tutorials on ROS2 with Python',
    license='GNU License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker= ros2_essential_python.talker:main',
            'listener= ros2_essential_python.listener:main',
            'service = ros2_essential_python.server:main',
            'async_client = ros2_essential_python.async_client:main',
            'sync_client = ros2_essential_python.sync_client:main',
        ],
    },

    
)
