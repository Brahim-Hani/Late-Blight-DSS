from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spray_scheduler_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include all files in launch/ in the final install
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hixy',
    maintainer_email='exphixydz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spray_scheduler = spray_scheduler_pkg.spray_scheduler:main',
        ],
    },
)

