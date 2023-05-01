from setuptools import setup

package_name = 'crazyflie_simulation'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/crazyflie_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/crazyflie_apartment.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/crazyflie.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimberly',
    maintainer_email='kimberly@bitcraze.io',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_webots_driver = crazyflie_simulation.crazyflie_webots_driver:main'
        ],
    },
)
