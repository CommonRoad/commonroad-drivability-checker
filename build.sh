#!/usr/bin/env bash

# Constants
#set -e
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

POSITIONAL=()
BASEDIR=$(pwd)
INSTALL="FALSE"
WHEEL="FALSE"
CGAL="FALSE"
S11N="FALSE"
NO_ROOT="FALSE"
PREFIX_STRING=""
JOBS=1
DOCS="FALSE"
COMMONROAD=""
PYTHONFILE=""
USAGE="
$(basename "$0") [options] -- builds the drivability checker.
Options:
    -h | --help   show this help ext
    -e PATH | --environment PATH   absolute path to the python environment
    -v VERSION | --version VERSION    python version
    -i | --install    python install after compiling
    -w | --wheel    create wheel package
    --cgal    install the third-party package cgal
    --serializer    install the third-party package s11n for pickle support
    -j COUNT    allowed job count for make -j ${JOBS}
    -d | --docs   creates the documentation while compiling
    -c PATH | --commonroad PATH  absolute path to the commonroad-io library
    --no-root   install libraries to the user home directory
"

# Functions
function print() {
  echo -e "${1}"
  if [ "${2}" == "-n" ]; then
    echo -e ""
  fi
}

function print_progress() {
  print "${GREEN}${1}${NC}" ${2}
}

function print_info() {
  print "${BLUE}${1}${NC}" ${2}
}

function print_error() {
  print "${RED}${1}${NC}" ${2}
}

function check_mandatory_args() {
  if [ "${ENVIRONMENT}" == "" ] || [ "${VERSION}" == "" ]; then
    print_error "The python environment and version must be defined!"
    print_info "${usage}"
    exit 1
  fi
}

function check_doc_args() {
  if [ "${DOCS}" == "TRUE" ] && [ "${COMMONROAD}" == "" ]; then
    print_error "The absolute path to commonroad-io must be defined!"
    print_info "${usage}"
    exit 1
  fi
}

function print_args() {
  print_info "" -n
  print_info "ENVIRONMENT PATH      = ${ENVIRONMENT}"
  print_info "PYTHON VERSION        = ${VERSION}"
  print_info "INSTALL AFTER COMPILE = ${INSTALL}"
  print_info "CREATE WHEEL PACKAGE  = ${WHEEL}"
  print_info "INSTALL CGAL          = ${CGAL}"
  print_info "NO ROOT               = ${NO_ROOT}"
  print_info "ALLOWED JOBS          = ${JOBS}"
  print_info "DOCS                  = ${DOCS}"
  print_info "COMMONROAD PATH       = ${COMMONROAD}" -n
}

function remove_folder() {
  for value in "$@"; do
    rm -rf ${value}
    print_info "Removed ${value}"
  done
}

function osx_command() {
  if [ "$(uname)" == "Darwin" ]; then
    ${@}
  fi
}

function linux_command() {
  if [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    ${@}
  fi
}

function require_sudo() {
  if [ "${NO_ROOT}" == "TRUE" ]; then
    ${@}
  else

    if [[ $EUID -ne 0 ]]; then
      print_info "Permission required, using root."
      sudo ${@}
    else
      ${@}
    fi
  fi
}

function set_pythonfile() {
  if [ -f ${ENVIRONMENT}/bin/python${VERSION} ]; then
    PYTHONFILE=${ENVIRONMENT}/bin/python${VERSION}
  else
    if [ -f ${ENVIRONMENT}/bin/python${VERSION}m ]; then
      PYTHONFILE=${ENVIRONMENT}/bin/python${VERSION}m
    else
      print_error "Could not find python interpreter!"
      exit 1
    fi
  fi
}

function epython() {
  ${PYTHONFILE} ${@}
}

function back_to_basedir() {
  cd ${BASEDIR}
}

function create_build_dir() {
  if [ "${1}" == "-r" ]; then
    rm -rf build
  fi
  mkdir build
  cd build
}

function fetch_submodules() {
  (
    set -e
    print_progress "Fetching submodules..." -n
    git submodule init
    git submodule update
    print_progress "Done!" -n
  )
}

function build_libccd() {
  (
    set -e
    print_progress "Building libccd..." -n
    cd third_party/libccd
    create_build_dir -r


    cmake -G "Unix Makefiles" $PREFIX_STRING -DENABLE_DOUBLE_PRECISION=ON -DBUILD_SHARED_LIBS=ON ..
    make -j ${JOBS}
    require_sudo make install

    print_progress "Done!" -n
    back_to_basedir
  )
}

function build_fcl() {
  (
    set -e
    print_progress "Building fcl..."
    print_info "Installing required packages for fcl..." -n
    cd third_party/fcl
    osx_command brew install eigen
    create_build_dir -r
    cmake $PREFIX_STRING ..
    make -j ${JOBS}
    require_sudo make install
    print_progress "Done!" -n
    back_to_basedir
  )
}

function build_s11n() {
  (
    set -e
    print_progress "Building s11n..."
    cd third_party/libs11n
    create_build_dir -r
    cmake $PREFIX_STRING .. -DCMAKE_BUILD_TYPE=Release
    make -j ${JOBS}
    require_sudo make install
    print_progress "Done!" -n
    back_to_basedir
  )
}

function install_cgal() {
  (
    set -e
    print_progress "Installing CGAL package..." -n
    osx_command brew install cgal cgal-qt5
    osx_command brew link cgal cgal-qt5
    linux_command require_sudo apt-get -y install libcgal-dev
    print_progress "Done!" -n
  )
}

function build_dc() {
  (
    set -e
    print_progress "Building drivability checker..." -n
    create_build_dir -r
    #epython -m pip install -r ../requirements.txt
    cmake $PREFIX_STRING -DADD_PYTHON_BINDINGS=TRUE -DPATH_TO_PYTHON_ENVIRONMENT="${ENVIRONMENT}" -DPYTHON_VERSION="${VERSION}" -DCMAKE_BUILD_TYPE=Release ..
    print_progress "Done!" -n
    osx_command sed -i '' 's!-lccd!/usr/local/lib/libccd.2.0.dylib!' python_binding/CMakeFiles/pycrcc.dir/link.txt
    make -j ${JOBS}
    print_progress "Done!" -n
    back_to_basedir
  )
}

function build_dc_with_docs() {
  (
    set -e
    print_progress "Building drivability checker..." -n
    create_build_dir -r
    mkdir -p doc/doxygen/html
    mkdir -p doc/doxygen/xml
    osx_command brew install doxygen pandoc graphviz
    osx_command brew link doxygen pandoc graphviz
    if [ "${NO_ROOT}" == "FALSE" ]; then
      linux_command require_sudo apt-get -y install doxygen python3-sphinx pandoc graphviz
    fi
    epython -m pip install -r ../requirements.txt
    #sed -i "s#../../commonroad-io/#${COMMONROAD}/#g" ../doc/conf.py
    cmake $PREFIX_STRING -DADD_PYTHON_BINDINGS=TRUE -DPATH_TO_PYTHON_ENVIRONMENT="${ENVIRONMENT}" -DPYTHON_VERSION="${VERSION}" -DCMAKE_BUILD_TYPE=Release -DBUILD_DOC=TRUE ..
    osx_command sed -i '' 's!-lccd!/usr/local/lib/libccd.2.0.dylib!' python_binding/CMakeFiles/pycrcc.dir/link.txt
    make -j ${JOBS}
    print_progress "Done!" -n
    back_to_basedir
  )
}

function install_dc() {
  (
    set -e
    print_progress "Installing as python package..." -n
    epython setup.py install
    print_progress "Done!" -n
  )
}

function install_dc_wheel() {
  (
    set -e
    print_progress "Installing as python package..." -n
    epython -m pip install dist/commonroad_drivability_checker-*.whl
    print_progress "Done!" -n
  )
}

function wheel_dc() {
  (
    set -e
    print_progress "Creating wheel package..." -n
    epython -m pip install wheel
    epython setup.py bdist_wheel
    print_progress "Done!" -n
  )
}

# Parse args
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
  -h | --help)
    echo -e "${USAGE}"
    exit 1
    ;;

  -e | --env)
    ENVIRONMENT="$2"
    ENVIRONMENT=${ENVIRONMENT%/}
    shift # past argument
    shift # past value
    ;;

  -v | --version)
    VERSION="$2"
    shift # past argument
    shift # past value
    ;;

  -i | --install)
    INSTALL="TRUE"
    shift # past argument
    ;;

  -w | --wheel)
    WHEEL="TRUE"
    shift # past argument
    ;;

  --cgal)
    CGAL="TRUE"
    shift # past argument
    ;;

  --serializer)
    S11N="TRUE"
    shift # past argument
    ;;

  --no-root)
    NO_ROOT="TRUE"
    shift # past argument
    ;;

  -j)
    JOBS="$2"
    shift # past argument
    shift # past value
    ;;

  -d | --docs)
    DOCS="TRUE"
    shift # past argument
    ;;

  -c | --commonroad)
    COMMONROAD="$2"
    COMMONROAD=${COMMONROAD%/}
    shift # past argument
    shift # past value
    ;;

  *) # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift              # past argument
    ;;
  esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

# Check args
check_mandatory_args
#check_doc_args
set_pythonfile
print_args

# Start building
remove_folder build dist *.egg-info *.so *.a
fetch_submodules
if [ "${NO_ROOT}" == "FALSE" ]; then
	linux_command require_sudo apt-get -y install build-essential cmake git wget unzip libboost-dev libboost-thread-dev
	linux_command require_sudo apt-get -y install libboost-test-dev libboost-filesystem-dev libeigen3-dev libomp-dev
else 
  PREFIX_STRING="-DCMAKE_PREFIX_PATH=$HOME -DCMAKE_INSTALL_PREFIX=$HOME"
fi
build_libccd
build_fcl
if [ "${S11N}" == "TRUE" ]; then
  build_s11n
fi
if [ "${NO_ROOT}" == "FALSE" ]; then
	if [ "${CGAL}" == "TRUE" ]; then
	  install_cgal
	fi
fi
if [ "${DOCS}" == "TRUE" ]; then
  build_dc_with_docs
else
  build_dc
fi
if [ "${WHEEL}" == "TRUE" ]; then
  wheel_dc
fi
if [ "${INSTALL}" == "TRUE" ]; then
  if [ "${WHEEL}" == "TRUE" ]; then
    install_dc_wheel
  else
    install_dc
  fi
fi
