import unittest as unittest
import os
import sys

if __name__ == "__main__":
    print(os.getcwd())
    errors = 0
    failures = 0
    tests = 0
    all_tests = unittest.TestLoader().discover(os.getcwd(), pattern='test_*.py')
    b = unittest.TextTestRunner().run(all_tests)
    failures += len(b.failures)
    errors += len(b.errors)
    tests += b.testsRun

    print(
        'Executed {} tests; got {} fails and {} errors'.format(tests, failures, errors)
    )
    if errors > 0 or failures > 0:
        sys.exit(1)
