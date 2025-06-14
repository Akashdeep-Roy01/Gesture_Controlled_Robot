from setuptools import find_packages, setup

package_name = 'gesture_to_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'moveit2'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='akashdeep.roy@rwth-aachen.de',
    description='TODO: package to convert hand gestures to suitable pose message',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_node = gesture_to_pose.gesture_node:main',
        ],
    },
)
