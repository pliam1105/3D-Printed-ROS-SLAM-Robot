from setuptools import setup

package_name = 'arduino_serial'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pliam1105',
    maintainer_email='pliam1105@gmail.com',
    description='A package with nodes for communicating with the Arduino controlling the motors/encoders',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_serial = arduino_serial.serial_nodes:main'
        ],
    },
)
