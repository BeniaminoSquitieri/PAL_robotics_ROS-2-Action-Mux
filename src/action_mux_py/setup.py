from setuptools import setup, find_packages

package_name = 'action_mux_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Pacchetto nodi per Action Server/Client che utilizza interfacce custom',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_server = action_mux_pkg.action_server:main',
            'action_client = action_mux_pkg.action_client:main',
            'generic_subscriber = action_mux_pkg.generic_subscriber:main',
            'string_publisher = action_mux_pkg.string_publisher:main',
        ],
    },
)
