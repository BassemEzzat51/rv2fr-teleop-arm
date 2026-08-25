from setuptools import find_packages, setup

package_name = 'rv2fr_teleop_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bassem',
    maintainer_email='bassemezzat165@gmail.com',
    description='PyQt6 teleoperation GUI for the Mitsubishi RV-2FR in gz sim.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'teleop_gui = rv2fr_teleop_gui.main_window:main',
        ],
    },
)
