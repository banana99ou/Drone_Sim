from setuptools import setup

package_name = 'dsim_viz'

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
    maintainer_email='banana99ou@kookmin.ac.kr',
    description='One-port viewer server: static files plus live state over SSE.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'viz_server = dsim_viz.viz_server:main',
        ],
    },
)
