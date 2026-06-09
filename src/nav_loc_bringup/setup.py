from setuptools import setup
from glob import glob

package_name = 'nav_loc_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],  # no python modules — this is a pure launch package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz',   glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='v-yuanjielu',
    maintainer_email='you@example.com',
    description='Top-level launch + rviz configs for Go2 nav/loc.',
    license='BSD-2-Clause',
    entry_points={'console_scripts': []},
)
