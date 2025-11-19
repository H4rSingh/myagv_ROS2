import os
from glob import glob
from setuptools import setup

package_name = 'myagv_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'hook'), glob('hook/*.sh')),
        *[(os.path.join('share', package_name, os.path.dirname(file_path)), [file_path]) 
          for file_path in glob(os.path.join('models', '**/*'), recursive=True) 
          if os.path.isfile(file_path)],        
    ],
    zip_safe=True,
    description='myagv Gazebo Package',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'command_timeout = myagv_gazebo.command_timeout:main'
        ],
    },
)                                                                                                                                                                                                                