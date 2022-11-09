eval "$('/Users/runner/miniconda3/bin/conda' 'shell.bash' 'hook')"
export MACOSX_DEPLOYMENT_TARGET=10.13
rm -rf commonroad_dc/*.so
rm -rf commonroad_dc/*.pyd
conda create -n common_9 python=3.9
conda activate common_9
pip install delocate
BUILD_JOBS=3 python setup.py build
python setup.py bdist_wheel
ls -la commonroad_dc
ls -la dist
cd dist
wheel_name=(commonroad_drivability_checker*cp39*macosx*.whl)
delocate-wheel -w fixed_wheels -v "${wheel_name[0]}"
ls -la fixed_wheels
cd ..
