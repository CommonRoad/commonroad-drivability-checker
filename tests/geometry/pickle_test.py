from commonroad_dc import pycrcc
from commonroad_dc import pycrccosy
import pickle

with open("reference_path_b.pic", "rb") as f:
    data_set = pickle.load(f)
reference_path = data_set['reference_path']

cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)

cosy.compute_and_set_curvature()

cosy_dump = pickle.dumps(cosy)

cosy2 = pickle.loads(cosy_dump)

cosy_dump2 = pickle.dumps(cosy2)

if(cosy_dump==cosy_dump2):
    print("test passed")
else:
    print("test failed")
