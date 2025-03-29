from setuptools import setup

package_name = 'grr_python_controllers'

import os
from glob import glob

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name, 'state_action/'), glob(os.path.join('state_actions', '*.*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='philip',
    maintainer_email='philip@randomsmiths.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':[ 
        	'drivetrain = grr_python_controllers.grr_drive_train:main',
            'state_machine = grr_python_controllers.state_machine:main'
        ],
    },
)
