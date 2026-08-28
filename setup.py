from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'lie_group_swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools','numpy-quaternion','numpy'],
    zip_safe=True,
    maintainer='Dimitria Silveria',
    maintainer_email='dimitriasilveria.ds@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encirclement = lie_group_swarm.encirclement_node:main',
            'crazy_circle = lie_group_swarm.crazy_circle:main',
            'agents_order = lie_group_swarm.agents_order:main',
            'circle_distortion = lie_group_swarm.circle_distortion:main',
            'full_reference = lie_group_swarm.full_reference:main',
        ],
    },
)
