from setuptools import setup, dist, find_packages


class BinaryDistribution(dist.Distribution):
    """ Make sure the setup.py will be a binary distribution. """

    def has_ext_modules(foo):
        return True


setup(
    name='commonroad-drivability-checker',
    version='2021.1',
    description='Drivability checker for CommonRoad scenarios.',
    url='https://commonroad.in.tum.de/',
    author='Technical University of Munich',
    author_email='commonroad-i06@in.tum.de',
    license='BSD',
    data_files=[('.', ['LICENSE'])],

    # Source
    distclass=BinaryDistribution,
    zip_safe=False,
    include_package_data=True,
    packages=['commonroad_dc'],
    package_data={'commonroad_dc': ['*.so']},

    # Requirements
    python_requires='>=3.6',
    install_requires=[
        'commonroad-io>=2020.3',
        'commonroad-vehicle-models>=1.0.0',
        'numpy>=1.19',
        'scipy>=1.4.1',
        'matplotlib>=3.2.2',
        'polygon3>=3.0.8',
        'shapely>=1.6.4',
        'triangle>=20200424',
    ],

    # Additional information
    classifiers=[
        "Programming Language :: C++",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "License :: OSI Approved :: BSD License",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
    ],
)
