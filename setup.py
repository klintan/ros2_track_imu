from setuptools import setup, find_packages

setup(
    name='ros2_track_imu',
    version='0.1.0',
    packages=find_packages(),
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'pyserial',
                      'transforms3d',
                      'numpy',
                      'pyyaml',
                      ],
    author='Andreas Klintberg',
    maintainer='Andreas Klintberg',
    keywords=['ROS2'],
    description='Package to publish a very simple IMU message from TrackIMU.',
    license='MIT',
    entry_points={
        'console_scripts': ['ros2_track_imu = ros2_track_imu.nodes.imu_node:main'],
    },
)
