eval "$('/C/Miniconda3/Scripts/conda.exe' 'shell.bash' 'hook')"
rm -rf commonroad_dc/*.pyd
conda create -n common_8 python=3.8
conda activate common_8
python setup.py build
python setup.py bdist_wheel