import math
import unittest
import commonroad_dc.geometry.util as geom_Util
import numpy as np
import pickle
import os


class TestGeometryUtil(unittest.TestCase):
    def setUp(self) -> None:
        try:
            # Local Test side
            with open(os.path.abspath('reference_path_b.pic'), 'rb') as path_file:
                data_set = pickle.load(path_file)
        except OSError as e:
            # CI Test side
            with open(os.path.abspath('geometry/reference_path_b.pic'), 'rb') as path_file:
                data_set = pickle.load(path_file)
        self.reference_path_test = data_set['reference_path']
        self.number_of_samples = len(self.reference_path_test)

        try:
            # Local Test side
            with open(os.path.abspath('reference_path_b_data.pic'), 'rb') as data_file:
                data_details = pickle.load(data_file)
        except OSError as e:
            # CI Test side
            with open(os.path.abspath('geometry/reference_path_b_data.pic'), 'rb') as data_file:
                data_details = pickle.load(data_file)
        self.polyline_length = data_details['polyline_length']
        self.path_length = data_details['path_length']
        self.curvature = data_details['curvature']
        self.orientation = data_details['orientation']

    def test_resample_polyline(self):
        reference_path_resampled_more_samples = len(geom_Util.resample_polyline(self.reference_path_test, 1.0))
        reference_path_resampled = len(geom_Util.resample_polyline(self.reference_path_test, 2.0))
        reference_path_resampled_less_samples = len(geom_Util.resample_polyline(self.reference_path_test, 3.0))

        self.assertGreater(reference_path_resampled_more_samples, self.number_of_samples, msg="Number of samples "
                                                                                              "should be larger")
        self.assertEqual(reference_path_resampled, self.number_of_samples, msg="Number of samples should be equal")
        self.assertLess(reference_path_resampled_less_samples, self.number_of_samples, msg="Number of samples should "
                                                                                           "be smaller")

    def test_resample_polyline_with_length_check(self):
        length_to_check = 2.0
        reference_path_resampled_length = len(geom_Util.resample_polyline_with_length_check(self.reference_path_test,
                                                                                            length_to_check))

        self.assertGreater(reference_path_resampled_length, self.number_of_samples, msg="The returned polyline should "
                                                                                        "have more samples")

    def test_compute_pathlength_from_polyline(self):
        returned_path_length = geom_Util.compute_pathlength_from_polyline(self.reference_path_test)
        self.assertEqual(self.number_of_samples, len(returned_path_length), msg='Polylines should be equally resampled')
        assert np.allclose(returned_path_length, self.path_length)

    def test_compute_polyline_length(self):
        returned_polyline_length = geom_Util.compute_polyline_length(self.reference_path_test)
        assert math.isclose(returned_polyline_length, self.polyline_length)

    def test_compute_curvature_from_polyline(self):
        returned_curvature = geom_Util.compute_curvature_from_polyline(self.reference_path_test)
        self.assertEqual(self.number_of_samples, len(returned_curvature), msg='Polylines should be equally resampled')
        assert np.allclose(returned_curvature, self.curvature)

    def test_compute_orientation_from_polyline(self):
        returned_orientation = geom_Util.compute_orientation_from_polyline(self.reference_path_test)
        self.assertEqual(self.number_of_samples, len(returned_orientation), msg='Polylines should be equally resampled')
        assert np.allclose(returned_orientation, self.orientation)

    def test_resample_polyline_python(self):
        self.assertGreaterEqual(self.number_of_samples, 2, msg="Polyline should have at least 2 points")
        returned_polyline = geom_Util.resample_polyline_python(self.reference_path_test, 2.0)
        test_check = True
        length_to_check = np.linalg.norm(returned_polyline[1] - returned_polyline[0])
        tolerance = 1e-1
        length_to_check_min = length_to_check - tolerance
        length_to_check_max = length_to_check + tolerance
        for i in range(1, len(returned_polyline)):
            length = np.linalg.norm(returned_polyline[i] - returned_polyline[i - 1])
            if length < length_to_check_min or length > length_to_check_max:
                test_check = False
                break
        self.assertEqual(test_check, True, msg="Polyline is not resampled with equidistant spacing")


if __name__ == '__main__':
    unittest.main()
