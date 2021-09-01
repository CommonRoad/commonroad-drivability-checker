import commonroad_dc.pycrccosy as pycrccosy
import numpy as np

from commonroad_dc.geometry.util import chaikins_corner_cutting, resample_polyline, \
    compute_curvature_from_polyline


class RefPathLengthException(Exception):
    pass


class CurvilinearCoordinateSystem(pycrccosy.CurvilinearCoordinateSystem):
    def __init__(self, ref_path, default_projection_domain_limit=25.0, eps=0.1, eps2=1e-4,
                 resample=True):
        if resample:
            ref_path = chaikins_corner_cutting(ref_path, 10)
            ref_path = resample_polyline(ref_path, 2.0)

        if len(ref_path) < 3:
            raise RefPathLengthException("Reference path length is invalid")

        super().__init__(ref_path, default_projection_domain_limit, eps, eps2)

    def _set_curvature(self, ccosy_ref_path_curvature: np.ndarray):

        # commonroad_dc.pycrccosy extends the original polyline with 3 segments(vertex in the beginning)
        # and with 2 segments(at the end) so we have to adjust the curvatre to this
        modified_curvature = [0.0] * 3 + list(ccosy_ref_path_curvature) + [0.0] * 2
        self.set_curvature(modified_curvature)

    def calc_and_set_curvature(self):
        curvature = compute_curvature_from_polyline(np.array(self.reference_path()))
        self._set_curvature(curvature)
