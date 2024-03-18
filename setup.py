import os
import re
import sys
import platform
import subprocess
import pathlib
import shutil
from setup_options import setup_options_dict

from sysconfig import get_paths

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

from distutils.version import LooseVersion

from commonroad_dc.__version__ import __version__

this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, 'README.md'), 'r', encoding='utf-8') as f:
    readme = f.read()


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

        # from find_libpython import find_libpython
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        default_python_include_dir = get_paths()['include']
        default_python_library = ""  # find_libpython()
        default_python_executable = sys.executable

        if setup_options_dict['PYTHON_INCLUDE_DIR'] != '':
            python_include_dir = setup_options_dict['PYTHON_INCLUDE_DIR']
        else:
            python_include_dir = default_python_include_dir

        if setup_options_dict['PYTHON_LIBRARY'] != '':
            python_library = setup_options_dict['PYTHON_LIBRARY']
        else:
            python_library = default_python_library

        if setup_options_dict['PYTHON_EXECUTABLE'] != '':
            python_executable = setup_options_dict['PYTHON_EXECUTABLE']
        else:
            python_executable = default_python_executable

        cmake_args = [
            "-DPYTHON_INCLUDE_DIR=" + python_include_dir,
            "-DPYTHON_LIBRARY=" + python_library,
            "-DPYTHON_EXECUTABLE=" + python_executable,
        ]

        # build documentation
        build_doc = 'OFF'
        if 'BUILD_DOC' in os.environ:
            build_doc = os.environ['BUILD_DOC']
            cmake_args += ['-DBUILD_DOC=' + build_doc]
        elif setup_options_dict['BUILD_DOC'] != '':
            build_doc = setup_options_dict['BUILD_DOC']
            cmake_args += ['-DBUILD_DOC=' + build_doc]
        else:
            cmake_args += ['-DBUILD_DOC=OFF']

        # add tests
        if setup_options_dict['ADD_TESTS'] != '':
            cmake_args += ['-DADD_TESTS=' + setup_options_dict['ADD_TESTS']]
        else:
            cmake_args += ['-DADD_TESTS=OFF']

        # Enable the non-free Triangle library

        if setup_options_dict['ADD_TRIANGLE'] != '':
            cmake_args += ['-DADD_TRIANGLE=' + setup_options_dict['ADD_TRIANGLE']]
        else:
            cmake_args += ['-DADD_TRIANGLE=OFF']

        # add python bindings
        if setup_options_dict['ADD_PYTHON_BINDINGS'] != '':
            cmake_args += ['-DADD_PYTHON_BINDINGS=' + setup_options_dict['ADD_PYTHON_BINDINGS']]
        else:
            cmake_args += ['-DADD_PYTHON_BINDINGS=ON']

        print(cmake_args)

        if setup_options_dict['DEBUG'] != '':
            self.debug = setup_options_dict['DEBUG']
        else:
            self.debug = False

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]

        dist_dir = os.path.abspath(os.path.join(self.build_temp, 'dist'))
        build_dir = os.path.abspath(os.path.join(self.build_temp, 'build'))

        install_dir = self.get_ext_fullpath(ext.name)
        extension_install_dir = pathlib.Path(install_dir).parent.joinpath(ext.name).resolve()

        for p in [dist_dir, build_dir]:
            if not os.path.exists(p):
                os.makedirs(p)

        cmake_args += ['-DCMAKE_INSTALL_PREFIX:PATH={}'.format(dist_dir)]

        build_args += ['--target', 'install']

        if ('BUILD_JOBS' in os.environ):
            build_args += ['--'] + ['-j'] + [os.environ['BUILD_JOBS']]

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=build_dir)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=build_dir)

        lib_dir = os.path.join(dist_dir, 'lib')
        if not os.path.exists(lib_dir):
            lib_dir = os.path.join(dist_dir, 'lib64')
        lib_python_dir = os.path.join(lib_dir, 'python')

        for file in os.listdir(lib_python_dir):
            self.copy_file(os.path.join(lib_python_dir, file), extension_install_dir)
            self.copy_file(os.path.join(lib_python_dir, file), os.path.join(os.getcwd(), 'commonroad_dc'))
        try:
            self.copy_file(os.path.join(lib_dir, 'libs11n.so'), extension_install_dir)
        except(Exception):
            pass

        try:
            self.copy_file(os.path.join(lib_dir, 'libs11n.dylib'), extension_install_dir)
        except(Exception):
            pass

        # copy to commonroad_dc/
        shutil.copy(os.path.join(lib_dir, 'libcrcc.a'), os.path.join(os.getcwd(), 'commonroad_dc'))
        shutil.copy(os.path.join(lib_dir, 'libcrccosy.a'), os.path.join(os.getcwd(), 'commonroad_dc'))
        shutil.copy(os.path.join(lib_dir, 'libgpc.a'), os.path.join(os.getcwd(), 'commonroad_dc'))
        try:
            shutil.copy(os.path.join(lib_dir, 'libtriangle.a'), os.path.join(os.getcwd(), 'commonroad_dc'))
        except(Exception):
            pass

        # copy documentation files to doc/build if Cmake Flag -DBUILD_DOC=='ON'
        if build_doc == 'ON':
            doc_target_dir = os.path.join(os.getcwd(), 'doc/build')
            if os.path.exists(doc_target_dir):
                shutil.rmtree(doc_target_dir)
            doc_source_dir = os.path.join(dist_dir, 'share/doc/DrivabilityChecker')
            shutil.copytree(doc_source_dir, doc_target_dir)

setup(
    name='commonroad-drivability-checker',
    version=__version__,
    description='Drivability checker for CommonRoad scenarios.',
    long_description_content_type='text/markdown',
    long_description=readme,
    url='https://commonroad.in.tum.de/drivability-checker',
    project_urls={
        'Documentation': 'https://cps.pages.gitlab.lrz.de/commonroad-drivability-checker/',
        'Forum': 'https://commonroad.in.tum.de/forum/c/commonroad-drivability-checker/',
        'Source': 'https://gitlab.lrz.de/tum-cps/commonroad-drivability-checker',
    },

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
    python_requires='>=3.7',
    install_requires=[
        'commonroad-io>=2022.3',
        'commonroad-vehicle-models>=3.0.0',
        'numpy>=1.19',
        'scipy>=1.4.1',
        'matplotlib>=3.2.2',
        'polygon3>=3.0.8',
        'shapely>=1.6.4',
        'setuptools>=62.1.0',
        'matplotlib>=3.2.2'
    ],

    # Additional information
    classifiers=[
        "Programming Language :: C++",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: BSD License",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
    ],
)
