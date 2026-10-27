from glob import glob

from setuptools import find_packages, setup

package_name = 'x650_keyframes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/x650_keyframes']),
        ('share/x650_keyframes', ['package.xml']),
        ('share/x650_keyframes/config', glob('config/*.*')),
        ('share/x650_keyframes/launch', glob('launch/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jian Gong',
    maintainer_email='164676+GoJian@users.noreply.github.com',
    description='Select and persist synchronized views and pose revisions.',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'node_name = x650_keyframes.module:main',
        ],
    },
)
