from setuptools import setup

package_name = 'dsim_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JHY',
    maintainer_email='banana99ou@gmail.com',
    description='Bridge from a space-time Bezier plan (x, y, z, t control points) to /drone/trajectory.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = dsim_planner.bridge_node:main',
        ],
    },
)
