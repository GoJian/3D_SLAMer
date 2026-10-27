from glob import glob

from setuptools import find_packages, setup

package_name = 'x650_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/x650_sim']),
        ('share/x650_sim', ['package.xml']),
        ('share/x650_sim/config', glob('config/*.*')),
        ('share/x650_sim/launch', glob('launch/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jian Gong',
    maintainer_email='164676+GoJian@users.noreply.github.com',
    description='PX4 SITL harness: fake VIO with fault injection, offboard test mission, evaluation.',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_vio = x650_sim.fake_vio:main',
            'x650_sim_mission = x650_sim.mission:main',
            'x650_sim_flight_test = x650_sim.flight_test:main',
            'x650_sim_evaluate = x650_sim.sim_eval:main',
        ],
    },
)
