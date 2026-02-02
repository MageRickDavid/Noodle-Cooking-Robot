from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'sequence'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cooker',
    maintainer_email='ricardo.ortizpozo@luxpmsoft.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        #'sequence=sequence.sequence:main',
        #'sequence_algorithm=sequence.sequence_algorithm:main',
        #'sequence_algorithm_damaged=sequence.sequence_algorithm_damaged:main',
        'sequence_optimal=sequence.sequence_optimal:main',
        'sequence_final=sequence.sequence_final:main',
        ],
    },
)
