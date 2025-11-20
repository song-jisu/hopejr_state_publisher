from setuptools import find_packages, setup

package_name = 'hopejr_state_publisher_gui'

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
    maintainer='songjisu',
    maintainer_email='sjs15290@knu.ac.kr',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hopejr_state_publisher_gui = hopejr_state_publisher_gui.hopejr_state_publisher_gui:main',
        ],
    },
)
