from setuptools import setup
import os
from glob import glob

package_name = 'single_inverted'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'),
         glob('launch/*launch.[pxy][yma]*')),
         (os.path.join('share', package_name, 'config/'),
         glob('config/**'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skully',
    maintainer_email='boliyadeep2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamics_sim = single_inverted.dynamics_sim:main',
            'dynamics_sim2 = single_inverted.dynamics_sim2:main',
            'controller_pend = single_inverted.controller_pend:main',
            'controller2_pend = single_inverted.controller2_pend:main'
        ],
    },
)
