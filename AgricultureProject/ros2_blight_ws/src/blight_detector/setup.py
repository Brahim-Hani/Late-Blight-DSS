from setuptools import setup

package_name = 'blight_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hixy',
    maintainer_email='exphixydz@gmail.com',
    description='Real-time blight detector node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blight_detector = blight_detector.blight_detector:main',
        ],
    },
)

