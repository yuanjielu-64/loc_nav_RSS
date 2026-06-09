from setuptools import setup
from glob import glob

package_name = 'nav_loc_localization'

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
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/dae', glob('dae/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='v-yuanjielu',
    maintainer_email='you@example.com',
    description='Localization for Go2.',
    license='BSD-2-Clause',
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = nav_loc_localization.odom_tf_broadcaster:main',
            'odom_relay = nav_loc_localization.odom_relay:main',
            'joint_state_bridge = nav_loc_localization.joint_state_bridge:main',
            'cloud_restamp = nav_loc_localization.cloud_restamp:main',
        ],
    },
)
