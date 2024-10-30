from setuptools import setup
from setuptools import find_packages

package_name = 'swarm_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='todo@todo.todo',
    description='Python package for turtlesim follower-leader swarm challenge.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_follower = swarm_challenge.turtle_follower:main'
        ]})
