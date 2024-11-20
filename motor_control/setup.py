from setuptools import setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'motor_control.motor_control_node',  # Keyboard-controlled version
        'motor_control.motor_control_joy',  # Joystick-controlled version
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
            'motor_control = motor_control.motor_control_node:main',  # Entry point for keyboard control
            'motor_control_joy = motor_control.motor_control_joy:main',  # Entry point for joystick control
        ],
    },
)
