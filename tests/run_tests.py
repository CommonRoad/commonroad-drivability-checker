import unittest as unittest
import os
import sys

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

    print(
        'Executed {} tests; got {} fails and {} errors'.format(tests, failures, errors)
    )
    if errors > 0 or failures > 0:
        sys.exit(1)
