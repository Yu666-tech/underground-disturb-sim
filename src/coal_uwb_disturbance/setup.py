from setuptools import setup

package_name = 'coal_uwb_disturbance'

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
    maintainer='yuhaotian',
    maintainer_email='yuhaotian@example.com',
    description='UWB NLOS/multipath disturbance module (B).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'uwb_node = coal_uwb_disturbance.node:main',
        ],
    },
)
