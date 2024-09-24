from setuptools import setup

package_name = 'pf_localisation'

setup(
    name=package_name,
    version='3.0.0',
    packages=['pf_localisation'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Alderman Webb',
    maintainer_email='s2186366@ed.ac.uk',
    description='Particle filter assignment for localisation in a map.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pf_node = pf_localisation.node:main',
        ]
    }
)
