from glob import glob

from setuptools import find_packages, setup

package_name = 'argus_localisation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ppatel',
    maintainer_email='pate2293@purdue.edu',
    description='The localisation pipeline for ARGUS: shrink, detect, crop, project, and the node that runs it.',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'loc_node = argus_localisation.loc_node:main',
        ],
    },
)
