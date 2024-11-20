from setuptools import setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'motor_control.motor_control_node',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Motor control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_node = motor_control.motor_control_node:main',
        ],
    },
)

