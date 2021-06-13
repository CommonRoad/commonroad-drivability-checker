import os
import re
import sys
import platform
import subprocess
import pathlib

from setuptools import setup, dist, find_packages, Extension
from setuptools.command.build_ext import build_ext

from distutils.version import LooseVersion

class BinaryDistribution(dist.Distribution):
    """ Make sure the setup.py will be a binary distribution. """

    def has_ext_modules(foo):
        return True

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.16':
                raise RuntimeError("CMake >= 3.16 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cmake_args = [
            "-DADD_PYTHON_BINDINGS=TRUE",
            "-DADD_TESTS=OFF",
            "-DBUILD_DOC=OFF"
          ]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]



        dist_dir = os.path.abspath(os.path.join(self.build_temp, 'dist'))
        build_dir = os.path.abspath(os.path.join(self.build_temp, 'build'))
        lib_python_dir = os.path.join(dist_dir, 'lib', 'python')
        install_dir = self.get_ext_fullpath(ext.name)
        extension_install_dir = pathlib.Path(install_dir).parent.joinpath(ext.name).resolve()

        for p in [dist_dir, build_dir]:
            if not os.path.exists(p):
                os.makedirs(p)

        cmake_args += [ '-DCMAKE_INSTALL_PREFIX:PATH={}'.format(dist_dir) ]

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=build_dir)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=build_dir)

        for file in os.listdir(lib_python_dir):
            self.copy_file(os.path.join(lib_python_dir, file), extension_install_dir)



setup(
    name='commonroad-drivability-checker',
    version='2021.1',
    description='Drivability checker for CommonRoad scenarios.',
    url='https://commonroad.in.tum.de/',
    author='Technical University of Munich',
    author_email='commonroad@lists.lrz.de',
    license='BSD',
    data_files=[('.', ['LICENSE'])],

    # Source
    zip_safe=False,
    include_package_data=True,
    packages=['commonroad_dc'],

    ext_modules=[CMakeExtension("commonroad_dc")],
    cmdclass={"build_ext": CMakeBuild},

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
