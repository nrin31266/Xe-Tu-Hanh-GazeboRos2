from setuptools import setup

package_name = 'cd2_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Package điều khiển xe tự hành né vật cản',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avoidance_node = cd2_control.avoidance_node:main',
            'simple_radar_gui = cd2_control.simple_radar_gui:main',
        ],
    },
)
