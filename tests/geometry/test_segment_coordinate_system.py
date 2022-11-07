import unittest
# from commonroad_dc import pycrcc
from commonroad_dc import pycrccosy
import numpy as np
import matplotlib.pyplot as plt
import pickle
from commonroad_dc.geometry.util import chaikins_corner_cutting, resample_polyline, \
    compute_curvature_from_polyline


class TestCurvilinearCoordinateSystem(unittest.TestCase):
    def setUp(self):
        self.show_plots = False

    def test_create_aa_collision_object_1(self):
        reference_path = np.array([[-1.2, 0.0],
                                   [0.0, 0.0],
                                   [3.0, 0.5],
                                   [4.5, 0.0],
                                   [6.5, 0.0]])
        cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
        boundary, triangle_mesh = cosy.convert_rectangle_to_cartesian_coords(
            0.5, 5.5, -2.0, 3.0)
        boundary = np.array(boundary)

        if self.show_plots:
            fig = plt.figure()
            ax1 = fig.add_subplot(2, 1, 1, adjustable='box', aspect=1.0)
            plt.plot(boundary[:, 0], boundary[:, 1], '*-g', zorder=50)
            plt.plot(reference_path[:, 0], reference_path[:, 1], '-r')
            plt.autoscale()
            plt.show()

    def test_transform_line_segment_to_cartesian_coords(self):
        try:
            with open("reference_path_b.pic", "rb") as f:
                data_set = pickle.load(f)
        except FileNotFoundError:
            with open("geometry/reference_path_b.pic", "rb") as f:
                data_set = pickle.load(f)
        reference_path = data_set['reference_path']

        cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
        x = np.linspace(80, 100, 200)
        y = 1.0

        s = list()
        l = list()
        for x_ in x:
            s_, l_ = cosy.convert_to_cartesian_coords(x_, y)
            s.append(s_)
            l.append(l_)
        if self.show_plots:
            plt.plot(s, l, '*-r', linewidth=2)
            plt.plot(reference_path[:, 0], reference_path[:, 1])
            plt.autoscale()
            plt.axis('equal')
            plt.show()

    def test_transform_line_segment_to_curvilinear_coords(self):
        try:
            with open("reference_path_b.pic", "rb") as f:
                data_set = pickle.load(f)
        except FileNotFoundError:
            with open("geometry/reference_path_b.pic", "rb") as f:
                data_set = pickle.load(f)
        reference_path = data_set['reference_path']

        cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
        x = np.linspace(-96, -70, 200)
        y = 4095

        plt.plot(reference_path[:, 0], reference_path[:, 1])
        for x_ in x:
            plt.plot(x_, y, '*y', alpha=0.7, linewidth=5)
        plt.autoscale()
        plt.axis('equal')
        plt.show()

        s = list()
        l = list()
        for x_ in x:
            s_, l_ = cosy.convert_to_curvilinear_coords(x_, y)
            s.append(s_)
            l.append(l_)
        if self.show_plots:
            plt.plot(s, l, '*-r', linewidth=2)
            plt.autoscale()
            plt.axis('equal')
            plt.show()

    def test_convert_group_of_points_from_cartesian_to_cv_and_back(self):
        # load reference path
        try:
            with open("reference_path_b.pic", "rb") as f:
                data_set = pickle.load(f)
        except FileNotFoundError:
            with open("geometry/reference_path_b.pic", "rb") as f:
                data_set = pickle.load(f)
        reference_path = data_set['reference_path']

        max_curvature = 0.4
        while max_curvature > 0.2:
            reference_path = np.array(chaikins_corner_cutting(reference_path))
            reference_path = resample_polyline(reference_path, 1.0)
            max_curvature = max(abs(compute_curvature_from_polyline(reference_path)))

        cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
        projection_domain = np.array(cosy.projection_domain())

        # load points
        try:
            with open("segment_coordinate_system_reference_path_b_points_a.pic", "rb") as f:
                data_set = pickle.load(f)
        except FileNotFoundError:
            with open("geometry/segment_coordinate_system_reference_path_b_points_a.pic", "rb") as f:
                data_set = pickle.load(f)
        x = data_set['x']
        y = data_set['y']

        # FIXME convert_list_of_points_to_curvilinear_coords will silently drop any points
        # outside the projection domain! In order to compare the original and converted
        # cartesian points, we need to skip any points outside the projection domain.
        # Otherwise the point indices won't line up when comparing the arrays.
        points = []
        for xv,yv in zip(x,y):
            if cosy.cartesian_point_inside_projection_domain(xv, yv):
                points += [[xv, yv]]

        cartesian_points = np.array(points)

        # Sanity check that we skipped points outside the projection domain
        # Exact number based on test data
        self.assertEqual(cartesian_points.shape, (19644, 2))

        # Convert points to Curvilinear
        p_curvilinear = np.array(cosy.convert_list_of_points_to_curvilinear_coords(cartesian_points, 4))

        # Convert points back to Cartesian
        p_cartesian = np.array(cosy.convert_list_of_points_to_cartesian_coords(p_curvilinear, 4))

        self.assertEqual(cartesian_points.shape, p_curvilinear.shape)
        self.assertEqual(cartesian_points.shape, p_cartesian.shape)

        # Compare original and converted cartesian coordinates
        np.testing.assert_allclose(p_cartesian, cartesian_points, atol=1e-3, rtol=0)

        # Verbose Comparison (disabled by default)
        verbose = False

        if verbose:
            number_of_failed_data_points = 0
            for i in range(cartesian_points.shape[0]):
                print('Number of iterations: '+str(len(x)))
                print("\nid:{} ".format(i))

                x_ref = cartesian_points[i,0]
                y_ref = cartesian_points[i,1]

                try:
                    # Points to be tested x_, y_
                    x_, y_ = p_cartesian[i][0], p_cartesian[i][1]
                except Exception as e:
                    plt.plot(reference_path[:, 0], reference_path[:, 1])
                    plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
                    plt.plot(x_ref, y_ref, '*k', linewidth=5)
                    print(e)
                    break
                try:
                    # We test x_, y_ against original dataset x and y
                    print('Calculated: (', x_, ', ', y_, ') - Real: (', x_ref, ', ', y_ref, ')')
                    np.testing.assert_allclose(x_, x_ref, atol=1e-3, rtol=0)
                    np.testing.assert_allclose(y_, y_ref, atol=1e-3, rtol=0)
                except Exception as e:
                    print(e)
                    plt.plot(reference_path[:, 0], reference_path[:, 1])
                    plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
                    plt.plot(x_ref, y_ref, '*g', linewidth=5)
                    plt.plot(x_, y_, '*r', linewidth=5)
                    number_of_failed_data_points += 1

            print("Number of failed data points: {} -> {}%".format(number_of_failed_data_points,
                                                                (number_of_failed_data_points*100)/len(x)))

            if self.show_plots:
                #draw_object_ccosy(cosy.get_segment_list())
                plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
                plt.autoscale()
                plt.axis('equal')
                plt.show()

    def test_determine_subset_of_polygon_in_projection_domain(self):
        # load reference path
        try:
            with open("reference_path_b.pic", "rb") as f:
                data_set = pickle.load(f)
        except FileNotFoundError:
            with open("geometry/reference_path_b.pic", "rb") as f:
                data_set = pickle.load(f)
        reference_path = data_set['reference_path']

        cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
        projection_domain = np.array(cosy.projection_domain())

        aabb = [np.array([-140, 4077]),
                np.array([-140, 4105]),
                np.array([-114, 4105]),
                np.array([-114, 4077])]
        aabb = np.array(aabb)

        res = cosy.determine_subset_of_polygon_within_projection_domain(aabb)
        res = np.concatenate(res)

        if self.show_plots:
            plt.plot(reference_path[:, 0], reference_path[:, 1])
            plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
            plt.plot(aabb[:, 0], aabb[:, 1], '-r')
            plt.plot(res[:, 0], res[:, 1], '-g')
            plt.autoscale()
            plt.axis('equal')
            plt.show()

    def test_determine_subsets_of_multi_polygons_within_projection_domain(self):
        # load reference path
        try:
            with open("reference_path_b.pic", "rb") as f:
                data_set = pickle.load(f)
        except FileNotFoundError:
            with open("geometry/reference_path_b.pic", "rb") as f:
                data_set = pickle.load(f)
        reference_path = data_set['reference_path']

        cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
        projection_domain = np.array(cosy.projection_domain())

        aabb = [np.array([-140, 4077]),
                np.array([-140, 4105]),
                np.array([-114, 4105]),
                np.array([-114, 4077])]
        aabb = np.array(aabb)
        poly = [np.array([-100, 4077]),
                np.array([-95, 4077]),
                np.array([-95, 4105]),
                np.array([-73, 4105]),
                np.array([-73, 4115]),
                np.array([-100, 4115]),
                np.array([-100, 4077])]
        poly.reverse()
        poly = np.array(poly)

        polygons = [aabb.tolist(), poly.tolist()]
        polygon_groups = [0, 1]
        clipped_polygon_all, clipped_polygon_groups_all = cosy.determine_subsets_of_multi_polygons_within_projection_domain(
            polygons, polygon_groups, 4)

        if self.show_plots:
            plt.plot(reference_path[:, 0], reference_path[:, 1])
            plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
            plt.plot(aabb[:, 0], aabb[:, 1], '-r')
            plt.plot(poly[:, 0], poly[:, 1], '-r')

            for r in clipped_polygon_all:
                r_array = np.array(r)
                plt.plot(r_array[:, 0], r_array[:, 1], '-g')

            plt.autoscale()
            plt.axis('equal')
            plt.show()


if __name__ == '__main__':
    unittest.main()
