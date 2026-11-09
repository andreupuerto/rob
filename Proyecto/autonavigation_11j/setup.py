from setuptools import find_packages, setup

package_name = 'autonavigation_11j'

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
    maintainer='rob',
    maintainer_email='rob@todo.todo',
    description='Navegacion autonoma de un TurtleBot3 en tres fases con Bug2, SLAM y acoplamiento',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'autonavigation = autonavigation_11j.autonavigation:main'
        ],
    },
)
