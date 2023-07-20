eval "$('/Users/runner/miniconda3/bin/conda' 'shell.bash' 'hook')"
conda create -n common_9_test python=3.9
conda activate common_9_test
pip install --upgrade pip
cd dist
cd fixed_wheels
wheel_name=(commonroad_drivability_checker*cp39*macosx*.whl)
pip install "${wheel_name[0]}"
cd ../../
cd tests
cd collision
python "pickle_test.py"
python "collision_unit_tests.py"
cd ../geometry
python "test_pickle.py"
