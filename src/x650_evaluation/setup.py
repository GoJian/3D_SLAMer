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
            'x650_timestamp_audit = x650_evaluation.timestamp_audit:main',
            'x650_synthetic_session = x650_evaluation.synthetic:main',
            'x650_evaluate = x650_evaluation.evaluate:main',
            'x650_replay = x650_evaluation.replay:main',
        ],
    },
)
