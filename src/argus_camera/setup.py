from glob import glob

from setuptools import find_packages, setup

package_name = 'argus_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/modes.yaml', 'config/robot.yaml']),
        ('share/' + package_name + '/config/calibration', glob('config/calibration/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='ppatel',
    maintainer_email='pate2293@purdue.edu',
    description='One camera driver for ARGUS: FrameSource library and node.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'camera_node = argus_camera.camera_node:main',
            'camera_bench = argus_camera.bench:main',
        ],
    },
)
