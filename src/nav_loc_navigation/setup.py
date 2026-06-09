from setuptools import setup
from glob import glob

package_name = 'nav_loc_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='v-yuanjielu',
    maintainer_email='you@example.com',
    description='Navigation (Nav2) wrapper for Go2.',
    license='BSD-2-Clause',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_sport = nav_loc_navigation.cmd_vel_to_sport:main',
            'estop = nav_loc_navigation.estop:main',
            'global_path_provider = nav_loc_navigation.global_path_provider:main',
        ],
    },
)
