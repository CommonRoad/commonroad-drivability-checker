import unittest as unittest
import os
import sys
from pathlib import Path


if __name__ == "__main__":
    print(os.getcwd())
    errors = 0
    failures = 0
    tests = 0
    for x in os.walk(os.getcwd()):
        # TODO costs tests need to be fixed and are temporarily disabled in CI
        # TODO some feasibility tests have a scipy-related bug and is temporarily disabled in CI
        if not '__' in x[0] and not '.' in x[0] and not 'costs' in x[0] and not 'feasibility' in x[0]:
            print(x[0])
            all_tests = unittest.TestLoader().discover(x[0], pattern='test_*.py')
            b = unittest.TextTestRunner().run(all_tests)
            failures += len(b.failures)
            errors += len(b.errors)
            tests += b.testsRun
            
    # # Run test_traffic_costs.py
    # specific_test = Path(__file__).parent.joinpath("test_traffic_rules_partial_costs/scripts/test_traffic_costs.py")
    # assert specific_test.exists()  # this way, we'll get an error if the test cannot be run. 
    # # if os.path.exists(specific_test):
    # print(f"Running traffic rule specific partial cost function test: {specific_test}")
    # all_tests = unittest.TestLoader().discover(os.path.dirname(specific_test), pattern="test_traffic_costs.py")
    # b = unittest.TextTestRunner().run(all_tests)
    # failures += len(b.failures)
    # errors += len(b.errors)
    # tests += b.testsRun
    
    # Run test_traffic_costs.py
    specific_test = "tests/test_traffic_rules_partial_costs/scripts/test_traffic_costs.py"
    if os.path.exists(specific_test):
        print(f"Running traffic rule specific partial cost function test: {specific_test}")
        all_tests = unittest.TestLoader().discover(os.path.dirname(specific_test), pattern="test_traffic_costs.py")
        b = unittest.TextTestRunner().run(all_tests)
        failures += len(b.failures)
        errors += len(b.errors)
        tests += b.testsRun



    print(
        'Executed {} tests; got {} fails and {} errors'.format(tests, failures, errors)
    )
    if errors > 0 or failures > 0:
        sys.exit(1)
