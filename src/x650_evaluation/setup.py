from glob import glob

from setuptools import find_packages, setup

package_name = 'x650_evaluation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/x650_evaluation']),
        ('share/x650_evaluation', ['package.xml']),
        ('share/x650_evaluation/config', glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jian Gong',
    maintainer_email='164676+GoJian@users.noreply.github.com',
    description='Alignment, metrics, runtime reports, fault injection.',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'node_name = x650_evaluation.module:main',
        ],
    },
)
