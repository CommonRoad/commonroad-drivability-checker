import os
import re
import sys
import platform
import subprocess
import pathlib


from sysconfig import get_paths

from setuptools import setup, dist, find_packages, Extension
from setuptools.command.build_ext import build_ext

from distutils.version import LooseVersion


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
    
        #from find_libpython import find_libpython
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep
            
        default_python_include_dir=get_paths()['include']
        default_python_library=""#find_libpython()
        default_python_executable=sys.executable
	
        if('PYTHON_INCLUDE_DIR' in os.environ):
            python_include_dir=os.environ['PYTHON_INCLUDE_DIR']
        else:
            python_include_dir=default_python_include_dir
	    
        if('PYTHON_LIBRARY' in os.environ):
            python_library=os.environ['PYTHON_LIBRARY']
        else:
            python_library=default_python_library
	   
        if('PYTHON_EXECUTABLE' in os.environ):
            python_executable=os.environ['PYTHON_EXECUTABLE']
        else:
            python_executable=default_python_executable
	     
        
        cmake_args = [
            "-DADD_PYTHON_BINDINGS=TRUE",
            "-DADD_TESTS=OFF",
            "-DBUILD_DOC=OFF",
            "-DPYTHON_INCLUDE_DIR="+python_include_dir,
            "-DPYTHON_LIBRARY="+python_library,
            "-DPYTHON_EXECUTABLE="+python_executable,	
          ]
          
        print(cmake_args)

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

        install_dir = self.get_ext_fullpath(ext.name)
        extension_install_dir = pathlib.Path(install_dir).parent.joinpath(ext.name).resolve()

        for p in [dist_dir, build_dir]:
            if not os.path.exists(p):
                os.makedirs(p)

        cmake_args += [ '-DCMAKE_INSTALL_PREFIX:PATH={}'.format(dist_dir) ]
        
        import multiprocessing
        
        build_args +=['--target','install']
        
        #install_args=build_args
        
        if('CMAKE_BUILD_PARALLEL_LEVEL' in os.environ):
            build_args += ['--']+['-j']+[os.environ['CMAKE_BUILD_PARALLEL_LEVEL']]

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=build_dir)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=build_dir)

        lib_dir=os.path.join(dist_dir, 'lib')
        if not os.path.exists(lib_dir):
            lib_dir=os.path.join(dist_dir, 'lib64')
        lib_python_dir = os.path.join(lib_dir, 'python')
        

        for file in os.listdir(lib_python_dir):
            self.copy_file(os.path.join(lib_python_dir, file), extension_install_dir)
        try:
            self.copy_file(os.path.join(lib_dir,'libs11n.so'), extension_install_dir)      
        except(Exception):
            pass

        try:
            self.copy_file(os.path.join(lib_dir,'libs11n.dylib'), extension_install_dir)
        except(Exception):
            pass


setup(
    name='commonroad-drivability-checker',
    version='2021.3',
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
    setup_requires=['find_libpython'],
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
