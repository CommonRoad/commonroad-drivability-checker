eval "$('/C/Miniconda3/Scripts/conda.exe' 'shell.bash' 'hook')"
conda create -n common_8_test python=3.8
conda activate common_8_test
pip3 install wheel
wheel_name=(dist/commonroad_drivability_checker*cp38*win*.whl)
pip3 install "${wheel_name[0]}"
cd tests
cd collision
python "pickle_test.py"
python "collision_unit_tests.py"
cd ../geometry
python "test_pickle.py"
