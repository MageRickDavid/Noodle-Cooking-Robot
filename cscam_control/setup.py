from setuptools import find_packages, setup

package_name = 'cscam_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'cscam_control = cscam_control.cscam_control:main',
            'cscamZ_control = cscam_control.control_z_motor:main'
        ],
    },
)
