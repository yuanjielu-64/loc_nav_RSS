from setuptools import setup

package_name = 'go2_front_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/front_camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@local',
    description='Polls Go2 main board videohub (GetImageSample) and republishes as ROS Image.',
    license='BSD-2-Clause',
    entry_points={
        'console_scripts': [
            'front_camera_node = go2_front_camera.front_camera_node:main',
        ],
    },
)
