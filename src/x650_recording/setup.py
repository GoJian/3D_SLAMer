from glob import glob

from setuptools import find_packages, setup

package_name = 'x650_recording'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/x650_recording']),
        ('share/x650_recording', ['package.xml']),
        ('share/x650_recording/config', glob('config/*.*')),
        ('share/x650_recording/launch', glob('launch/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jian Gong',
    maintainer_email='164676+GoJian@users.noreply.github.com',
    description='Capture raw inputs, calibration, configuration and diagnostics into a replayable session.',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'x650_record = x650_recording.record:main',
        ],
    },
)
