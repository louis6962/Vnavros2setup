from setuptools import setup
package_name = 'f1tenth_teleop'
setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop_ackermann.launch.py']),
        ('share/' + package_name + '/config', ['config/teleop_twist_joy.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Joystick + Twist→Ackermann.',
    license='MIT',
    entry_points={'console_scripts': [
        'twist_to_ackermann = f1tenth_teleop.twist_to_ackermann:main',
    ]},
)
