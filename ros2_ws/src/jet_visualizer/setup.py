import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'jet_visualizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf*'))),
        # Include mesh files
        (os.path.join('share', package_name, 'meshes'),
            glob(os.path.join('meshes', '*'))),
        # Include RViz config
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahamed Raafiq',
    maintainer_email='ahamadraafiqq@gmail.com',
    description='F-15C Eagle jet visualization using RViz2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_tf_broadcaster = jet_visualizer.imu_tf_broadcaster:main',
        ],
    },
)
