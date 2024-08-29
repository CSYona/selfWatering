from setuptools import setup

package_name = 'auto_initial_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choi',
    maintainer_email='choi@todo.todo',
    description='A package for automatically setting the initial pose of a robot.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'auto_initial_pose = auto_initial_pose.auto_initial_pose:main'
        ],
    },
)
