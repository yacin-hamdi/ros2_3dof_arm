from setuptools import find_packages, setup

package_name = 'cobot_py'

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
    maintainer='moose',
    maintainer_email='yacin.ha9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "2dof_move_arm = cobot_py.2dof_move_arm:main",
            "3dof_move_arm = cobot_py.3dof_move_arm:main"
        ],
    },
)
