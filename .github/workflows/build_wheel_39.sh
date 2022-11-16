eval "$('/C/Miniconda3/Scripts/conda.exe' 'shell.bash' 'hook')"
rm -rf commonroad_dc/*.pyd
conda create -n common_9 python=3.9
conda activate common_9
python setup.py build
python setup.py bdist_wheel