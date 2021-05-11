import commonroad_dc.pycrccosy as pycrccosy
from commonroad_dc.geometry.util import chaikins_corner_cutting, resample_polyline

class RefPathLengthException(Exception):
    pass

class CurvilinearCoordinateSystem(pycrccosy.CurvilinearCoordinateSystem):
    def __init__(self, reference_path, default_projection_domain_limit=25.0, eps=0.1, eps2=1e-4, resample=True):
        if resample == True:
            ref_path = chaikins_corner_cutting(reference_path, 10)
            ref_path = resample_polyline(ref_path, 2.0)

        if (len(ref_path) < 3):
            raise RefPathLengthException("Reference path length is invalid")
        super().__init__(ref_path, default_projection_domain_limit, eps, eps2)
