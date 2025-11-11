from collision.collision_broadphase_tests import run_test as run_collision_broadphase_tests
from collision.pickle_test import run_test as run_pickle_test
from collision.trajectory_queries_test import run_test as run_test_trajectory_queries
from collision.trajectory_broadphase_tests import run_test as run_trajectory_broadphase_tests
from collision.trajectory_narrowphase_tests import run_test as run_trajectory_narrowphase_tests
from collision.trajectory_test_empty_sg import run_test as run_trajectory_test_empty_sg

has_error = False

print("Running collision broadphase tests...")
if run_collision_broadphase_tests() == True:
    exit(1)

print("Running pickle tests...")
if run_pickle_test() == True:
    exit(1)

print("Running trajectory query tests...")
if run_test_trajectory_queries() == True:
    exit(1)

print("Running trajectory broadphase tests...")
if run_trajectory_broadphase_tests() == True:
    exit(1)

print("Running trajectory narrowphase tests...")
if run_trajectory_narrowphase_tests() == True:
    exit(1)

print("Running trajectory queries for empty ShapeGroups tests...")
if run_trajectory_test_empty_sg() == True:
    exit(1)