eval "$('/C/Miniconda3/Scripts/conda.exe' 'shell.bash' 'hook')"
rm -rf commonroad_dc/*.pyd
conda create -n common_7 python=3.7
conda activate common_7
BUILD_JOBS=2 python setup.py build
python setup.py bdist_wheel
