from setuptools import find_packages, setup

package_name = 'turtle_sim_navigation'

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
    maintainer='devin',
    maintainer_email='diazdevin7@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_client = turtle_sim_navigation.move_turtle:main',
            'turtle_service = turtle_sim_navigation.set_direction_service:main',
            'square = turtle_sim_navigation.turtle_square:main',
            'triangle = turtle_sim_navigation.turtle_triangle:main',
            'avoid_obs = turtle_sim_navigation.turtle_obstacle_avoidance:main',
        ],
    },
)
