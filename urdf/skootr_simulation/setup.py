from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skootr_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'meshes/frame'), glob(os.path.join('meshes/frame', '*.stl'))),
        (os.path.join('share', package_name, 'meshes/legs'), glob(os.path.join('meshes/legs', '*.stl'))),
        (os.path.join('share', package_name, 'meshes/non_printed'), glob(os.path.join('meshes/non_printed', '*.stl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jack',
    maintainer_email='jack@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
        # 'launch': [
        #     'skootr = skootr_simulation.launch.skootr:generate_launch_description'
        # ],
    },
)
