from setuptools import find_packages, setup

package_name = 'simulation'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/worlds', ['worlds/hyflex_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/hyflex.urdf']))
data_files.append(('share/' + package_name + '/resource/meshes/hyflex', ['resource/meshes/hyflex/back_collision.stl']))
data_files.append(('share/' + package_name + '/resource/meshes/hyflex', ['resource/meshes/hyflex/back.dae']))
data_files.append(('share/' + package_name + '/resource/meshes/hyflex', ['resource/meshes/hyflex/base_link_collision.stl']))
data_files.append(('share/' + package_name + '/resource/meshes/hyflex', ['resource/meshes/hyflex/base_link.dae']))
data_files.append(('share/' + package_name + '/resource/meshes/hyflex', ['resource/meshes/hyflex/wheel_collision.stl']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)