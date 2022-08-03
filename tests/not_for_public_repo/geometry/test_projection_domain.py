import numpy as np
import commonroad_dc.pycrccosy as pycrccosy
from commonroad_dc.geometry.util import resample_polyline

arr = np.array([[136.1013, -52.64465],
                [136.9463, -53.99465],
                [138.9713, -55.94465],
                [141.0563, -57.19465],
                [144.0213, -58.59465],
                [147.1013, -59.64465],
                [152.3213, -60.39465],
                [163.6113, -58.04465]])
res = pycrccosy.CurvilinearCoordinateSystem(resample_polyline(arr))

arr = np.array([[64.1976, -38.03163],
                [63.55926413, -37.26187204],
                [62.92092826, -36.49211408],
                [62.4976, -35.98163]])
arr_resample = resample_polyline(arr)
res = pycrccosy.CurvilinearCoordinateSystem(arr_resample, 20., 0.1, 0.00000001)

first_point = np.asarray(arr[0])
last_point = np.asarray(arr[-1])

last_point_eps = last_point + (np.asarray(arr[-2]) - last_point) * 0.00001

res.convert_to_curvilinear_coords(last_point_eps[0], last_point_eps[1])

for ind, x in enumerate(arr):
    res.convert_to_curvilinear_coords(x[0], x[1])

a = 0
