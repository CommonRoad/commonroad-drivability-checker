eval "$('/C/Miniconda3/Scripts/conda.exe' 'shell.bash' 'hook')"
conda create -n common_7_test python=3.7
conda activate common_7_test
pip3 install wheel
wheel_name=(dist/commonroad_drivability_checker*cp37*win*.whl)
pip3 install "${wheel_name[0]}"
cd tests
cd collision
python "pickle_test.py"
python "collision_unit_tests.py"
cd ../geometry
python "test_pickle.py"



