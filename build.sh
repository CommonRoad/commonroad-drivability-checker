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
NO_ROOT="FALSE"
DOCS="OFF"
JOBS=1
COMMONROAD=""

function usage() {
    echo "$(basename "$0") [options] -- builds the drivability checker."
    echo "Options:"
    echo "    -h | --help                     show this help ext"
    echo "    -i | --install                  python install after compiling"
    echo "    -w | --wheel                    create wheel package"
    echo "    --cgal                          install the third-party package cgal"
    echo '    -j COUNT                        allowed job count for make -j ${JOBS}'
    echo "    -d | --docs                     creates the documentation while compiling"
    echo "    -c PATH | --commonroad PATH     absolute path to the commonroad-io library"
    echo "    --no-root                       install libraries to the user home directory"
}

# Functions
function print() {
  echo -e "${1}"
  if [ "${2}" == "-n" ]; then
    echo -e ""
  fi
}

function print_progress() {
  print "${GREEN}${1}${NC}" "${2}"
}

function print_info() {
  print "${BLUE}${1}${NC}" "${2}"
}

function print_error() {
  print "${RED}${1}${NC}" "${2}"
}

function print_args() {
  print_info "" -n
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
    rm -rf "${value}"
    print_info "Removed ${value}"
  done
}

function osx_command() {
  if [[ "	" == "darwin"* ]]; then
    "${@}"
  fi
}

function linux_command() {
  if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    "${@}"
  fi
}

function require_sudo() {
  if [ "${NO_ROOT}" == "TRUE" ]; then
    "${@}"
  else

    if [[ $EUID -ne 0 ]]; then
      print_info "Permission required, using root."
      sudo "${@}"
    else
      "${@}"
    fi
  fi
}

function back_to_basedir() {
  cd "${BASEDIR}"
}

function fetch_submodules() {
  (
    set -e
    print_progress "Fetching submodules..." -n
    if git submodule init ; then
        print_progress "Submodule init succeeded" -n
    else
	print_progress "Submodule init failed" -n
        exit 1
    fi
    if git submodule update ; then
        print_progress "Submodule update succeeded" -n
    else
	print_progress "Submodule update failed" -n
        exit 1
    fi
    print_progress "Done!" -n
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
    BUILD_JOBS="$JOBS" BUILD_DOC="$DOCS" python setup.py build
    print_progress "Done!" -n
    back_to_basedir
  )
}


function install_dc() {
  (
    set -e
    print_progress "Installing as python package..." -n
    #python setup.py install - bad way to install, may download beta packages
    python -m pip install .
    print_progress "Done!" -n
  )
}

function install_dc_wheel() {
  (
    set -e
    print_progress "Installing as python package..." -n
    python -m pip install dist/commonroad_drivability_checker-*.whl
    print_progress "Done!" -n
  )
}

function wheel_dc() {
  (
    set -e
    print_progress "Creating wheel package..." -n
    python -m pip install wheel
    python setup.py bdist_wheel
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

  --no-root)
    NO_ROOT="TRUE"
    shift # past argument
    ;;

  -j)
    JOBS="$2"
    shift # past argument
    shift # past value
    ;;

  -c | --commonroad)
    COMMONROAD="$2"
    COMMONROAD=${COMMONROAD%/}
    shift # past argument
    shift # past value
    ;;

  -d | --docs)
    DOCS="ON"
    shift
    ;;

  *) # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift              # past argument
    ;;
  esac
done
set -- "${POSITIONAL[@]}" # restore positional parameters

# Check args
print_args

# Start building
remove_folder build dist ./*.egg-info
fetch_submodules
if [ "${NO_ROOT}" == "FALSE" ]; then
	linux_command require_sudo apt-get -y install build-essential cmake git wget unzip libboost-dev libboost-thread-dev
	linux_command require_sudo apt-get -y install libboost-test-dev libboost-filesystem-dev libeigen3-dev libomp-dev
fi
if [ "${NO_ROOT}" == "FALSE" ]; then
	if [ "${CGAL}" == "TRUE" ]; then
	  install_cgal
	fi
fi
build_dc
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
