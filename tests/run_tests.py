import unittest as unittest
import os
import sys
from pathlib import Path



# Ensure CI finds `commonroad_dc`
project_root = os.path.abspath(os.path.dirname(__file__) + "/..")
sys.path.insert(0, project_root)



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
            
    # Run the traffic rule partial costs tests
    test_script_path = os.path.join(os.getcwd(), "test_traffic_rules_partial_costs/scripts/test_traffic_costs.py")
    assert os.path.exists(test_script_path), f"Error: Test script {test_script_path} not found!"
    print(f"Now running {test_script_path}")
    
    suite = unittest.defaultTestLoader.loadTestsFromName("test_traffic_rules_partial_costs.scripts.test_traffic_costs")
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    failures += len(result.failures)
    errors += len(result.errors)
    tests += result.testsRun

    print(
        'Executed {} tests; got {} fails and {} errors'.format(tests, failures, errors)
    )
    if errors > 0 or failures > 0:
        sys.exit(1)
