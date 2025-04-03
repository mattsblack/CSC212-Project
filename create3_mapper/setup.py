from setuptools import setup

package_name = 'create3_mapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qcc',
    maintainer_email='qcc@qcc.todo',
    description='ROS2 package for Create3 mapping with custom RViz configuration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create3_mapper = create3_mapper.create3_mapper:main'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Install the launch files in share/create3_mapper/launch
        ('share/' + package_name + '/launch', ['launch/rviz_launch.py']),
        # (Optional) Install the RViz config in share/create3_mapper/rviz
        ('share/' + package_name + '/rviz', ['rviz/create3_mapper.rviz']),
    ],
)

