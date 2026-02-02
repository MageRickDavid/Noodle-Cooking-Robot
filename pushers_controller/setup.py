from setuptools import find_packages, setup

package_name = 'pushers_controller'

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
    maintainer_email='cooker@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pushers_controller_200mm=pushers_controller.pushers_controller_200mm:main',
            'pushers_controller_services=pushers_controller.pushers_controller_services:main',
             'pushers_controller=pushers_controller.pushers_controller:main',
        ],
    },
)
