import unittest
from commonroad_dc import pycrcc
from commonroad_dc import pycrccosy
import numpy as np
import matplotlib.pyplot as plt
import pickle
from commonroad_dc.geometry.util import chaikins_corner_cutting, resample_polyline, \
    compute_curvature_from_polyline


class TestCurvilinearCoordinateSystem(unittest.TestCase):
    def setUp(self):
        pass

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

        fig = plt.figure()
        ax1 = fig.add_subplot(2, 1, 1, adjustable='box', aspect=1.0)
        plt.plot(boundary[:, 0], boundary[:, 1], '*-g', zorder=50)
        plt.plot(reference_path[:, 0], reference_path[:, 1], '-r')
        plt.autoscale()
        plt.show()

    def test_transform_line_segment_to_cartesian_coords(self):
        with open("reference_path_b.pic", "rb") as f:
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
        plt.plot(s, l, '*-r', linewidth=2)
        plt.plot(reference_path[:, 0], reference_path[:, 1])
        plt.autoscale()
        plt.axis('equal')
        plt.show()

    def test_transform_line_segment_to_curvilinear_coords(self):
        with open("reference_path_b.pic", "rb") as f:
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
        plt.plot(s, l, '*-r', linewidth=2)
        plt.autoscale()
        plt.axis('equal')
        plt.show()

    def test_convert_to_curvilinear_coordinates_and_back(self):
        # load reference path
        with open("reference_path_b.pic", "rb") as f:
            data_set = pickle.load(f)
        reference_path = data_set['reference_path']

        cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
        projection_domain = np.array(cosy.get_projection_domain())

        # # create new dataset
        # x_min = min(reference_path[:, 0])
        # x_max = max(reference_path[:, 0])
        # y_min = min(reference_path[:, 1])
        # y_max = max(reference_path[:, 1])
        #
        # dx = 20
        # dy = 20
        # l = 20000
        #
        # x = list()
        # y = list()
        # while l > 0:
        #     x_ = np.random.uniform(x_min-dx, x_max+dx)
        #     y_ = np.random.uniform(y_min-dy, y_max+dy)
        #     if cosy.cartesian_point_inside_projection_domain(x_, y_):
        #         l -= 1;
        #         x.append(x_)
        #         y.append(y_)
        #
        # PICFILE = 'segment_coordinate_system_reference_path_b_points_a.pic'
        # file = open(PICFILE, 'wb')
        # pickle.dump({
        #     'x': np.array(x),
        #     'y': np.array(y),
        # }, file)
        # file.close()
        #
        # plt.plot(reference_path[:, 0], reference_path[:, 1])
        # plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
        # plt.plot(x, y, '*y', alpha=0.7, linewidth=5)
        # plt.show()

        # load points
        with open("segment_coordinate_system_reference_path_b_points_a.pic", "rb") as f:
            data_set = pickle.load(f)
        x = data_set['x']
        y = data_set['y']

        number_of_failed_data_points = 0
        for i in range(0, len(x)):
            print("\nid:{} ".format(i))
            try:
                p, idx = cosy.convert_to_curvilinear_coords_and_get_segment_idx(x[i], y[i])
                x_, y_ = cosy.convert_to_cartesian_coords(p[0], p[1])
                x_ref = x[i]
                y_ref = y[i]
            except Exception as e:
                plt.plot(reference_path[:, 0], reference_path[:, 1])
                plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
                plt.plot(x[i], y[i], '*k', linewidth=5)
                print(e)
                continue
            # p_prime = (cosy.get_segment_list()[idx]).convert_to_cartesian_coords(
            #     p[0] - cosy.get_longitudinal_segment_positions()[idx], p[1])
            try:
                np.testing.assert_allclose(x_, x[i], atol=1e-3, rtol=0)
                np.testing.assert_allclose(y_, y[i], atol=1e-3, rtol=0)
            except Exception as e:
                print(e)
                plt.plot(reference_path[:, 0], reference_path[:, 1])
                plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
                plt.plot(x[i], y[i], '*g', linewidth=5)
                plt.plot(x_, y_, '*r', linewidth=5)
                number_of_failed_data_points += 1

        print("Number of failed data points: {} -> {}%".format(number_of_failed_data_points,
                                                               (number_of_failed_data_points*100)/len(x)))
        #draw_object_ccosy(cosy.get_segment_list())
        plt.autoscale()
        plt.axis('equal')
        plt.show()

    def test_convert_to_curvilinear_coordinates_and_back(self):
        # load reference path
        with open("reference_path_b.pic", "rb") as f:
            data_set = pickle.load(f)
        reference_path = data_set['reference_path']

        max_curvature = 0.4
        while max_curvature > 0.2:
            reference_path = np.array(chaikins_corner_cutting(reference_path))
            reference_path = resample_polyline(reference_path, 1.0)
            max_curvature = max(abs(compute_curvature_from_polyline(reference_path)))

        cosy = pycrccosy.CurvilinearCoordinateSystem(reference_path)
        projection_domain = np.array(cosy.projection_domain())

        # # create new dataset
        # x_min = min(reference_path[:, 0])
        # x_max = max(reference_path[:, 0])
        # y_min = min(reference_path[:, 1])
        # y_max = max(reference_path[:, 1])
        #
        # dx = 20
        # dy = 20
        # l = 20000
        #
        # x = list()
        # y = list()
        # while l > 0:
        #     x_ = np.random.uniform(x_min-dx, x_max+dx)
        #     y_ = np.random.uniform(y_min-dy, y_max+dy)
        #     if cosy.cartesian_point_inside_projection_domain(x_, y_):
        #         l -= 1;
        #         x.append(x_)
        #         y.append(y_)
        #
        # PICFILE = 'segment_coordinate_system_reference_path_b_smoothed_points_a.pic'
        # file = open(PICFILE, 'wb')
        # pickle.dump({
        #     'x': np.array(x),
        #     'y': np.array(y),
        # }, file)
        # file.close()
        #
        # plt.plot(reference_path[:, 0], reference_path[:, 1])
        # plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
        # plt.plot(x, y, '*y', alpha=0.7, linewidth=5)
        # plt.show()

        # load points
        with open("segment_coordinate_system_reference_path_b_smoothed_points_a.pic", "rb") as f:
            data_set = pickle.load(f)
        x = data_set['x']
        y = data_set['y']

        cart_points = np.array(list(zip(x, y)))

        p = cosy.convert_list_of_points_to_curvilinear_coords(cart_points, 4)
        number_of_failed_data_points = 0
        for i in range(0, len(x)):
            print("\nid:{} ".format(i))
            try:
                x_, y_ = cosy.convert_to_cartesian_coords(p[i][0], p[i][1])
            except Exception as e:
                plt.plot(reference_path[:, 0], reference_path[:, 1])
                plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
                plt.plot(x[i], y[i], '*k', linewidth=5)
                print(e)
                break
            # p_prime = (cosy.get_segment_list()[idx]).convert_to_cartesian_coords(
            #     p[0] - cosy.get_longitudinal_segment_positions()[idx], p[1])
            try:
                np.testing.assert_allclose(x_, x[i], atol=1e-3, rtol=0)
                np.testing.assert_allclose(y_, y[i], atol=1e-3, rtol=0)
            except Exception as e:
                print(e)
                plt.plot(reference_path[:, 0], reference_path[:, 1])
                plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
                plt.plot(x[i], y[i], '*g', linewidth=5)
                plt.plot(x_, y_, '*r', linewidth=5)
                number_of_failed_data_points += 1

        print("Number of failed data points: {} -> {}%".format(number_of_failed_data_points,
                                                               (number_of_failed_data_points*100)/len(x)))
        #draw_object_ccosy(cosy.get_segment_list())
        plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
        plt.autoscale()
        plt.axis('equal')
        plt.show()

    def test_determine_subset_of_polygon_in_projection_domain(self):
        # load reference path
        with open("reference_path_b.pic", "rb") as f:
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
        plt.plot(reference_path[:, 0], reference_path[:, 1])
        plt.plot(projection_domain[:, 0], projection_domain[:, 1], '-b')
        plt.plot(aabb[:, 0], aabb[:, 1], '-r')
        plt.plot(res[:, 0], res[:, 1], '-g')
        plt.autoscale()
        plt.axis('equal')
        plt.show()

    def test_determine_subsets_of_multi_polygons_within_projection_domain(self):
        # load reference path
        with open("reference_path_b.pic", "rb") as f:
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
