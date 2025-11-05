from setuptools import find_packages, setup

package_name = 'unknown_rrt'

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
    maintainer='sarah',
    maintainer_email='sarah.mcamhon16@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'geometric_planner = unknown_rrt.geometric_planner:main',
            # reusable RRT planner (for testing or reuse)
            'rrt_planner = unknown_rrt.rrt_planner:main',
        ],
    },
)
