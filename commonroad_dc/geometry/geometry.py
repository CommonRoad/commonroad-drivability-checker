import numpy as np
import commonroad_dc.pycrccosy as pycrccosy
from commonroad_dc.geometry.util import chaikins_corner_cutting, resample_polyline, compute_polyline_length,\
    compute_pathlength_from_polyline, compute_orientation_from_polyline


class RefPathLengthException(Exception):
    pass


class CurvilinearCoordinateSystem(pycrccosy.CurvilinearCoordinateSystem):
    """
    Wrapper class for pycrccosy.CurvilinearCoordinateSystem
    """

    def __init__(self, ref_path, default_projection_domain_limit=25.0, eps=0.1, eps2=1e-4, resample=True,
                 num_chaikins_corner_cutting=10, max_polyline_resampling_step=2.0):
        """
        Initializes a Curvilinear Coordinate System for a given reference path.
        :param ref_path Reference Path as 2D polyline in Cartesian coordinates
        :param default_projection_domain_limit maximum absolute distance in lateral direction of the projection domain\
         border from the reference path
        :param eps reduces the lateral distance of the projection domain border from the reference path
        :param eps2 if nonzero, add additional segments to the beginning (3 segments) and the end (2 segments) of the\
         reference path to enable the conversion of the points near the beginning and the end of the reference path
        :param resample: whether the reference path should be refined using chainkins algorithm.
        :param num_chaikins_corner_cutting: how often chaikins algorithm should be applied.
        :param max_polyline_resampling_step: maximum step size for polyline resampling.
        """
        if resample:
            ref_path = chaikins_corner_cutting(ref_path, num_chaikins_corner_cutting)
            length = compute_polyline_length(ref_path)
            polyline_resampling_step = min(max_polyline_resampling_step, length/num_chaikins_corner_cutting)
            ref_path = resample_polyline(ref_path, polyline_resampling_step)

        if len(ref_path) < 3:
            raise RefPathLengthException("Reference path length is invalid")

        # remove duplicated vertices in reference path
        _, idx = np.unique(ref_path, axis=0, return_index=True)
        ref_path = ref_path[np.sort(idx)]

        # initialize Curvilinear Coordinate System
        super().__init__(ref_path, default_projection_domain_limit, eps, eps2)
        # compute curvature
        super().compute_and_set_curvature()

        # initialize reference attributes
        self._reference = np.asarray(super().reference_path())
        self._ref_pos = compute_pathlength_from_polyline(self.reference)
        self._ref_curv = np.asarray(super().get_curvature())
        self._ref_curv_d = np.gradient(self._ref_curv, self._ref_pos)
        self._ref_theta = np.unwrap(compute_orientation_from_polyline(self.reference))

    def __getstate__(self):
        return(pycrccosy.CurvilinearCoordinateSystem.__getstate__(self),
               self.__dict__)

    def __setstate__(self, state: tuple):
        pycrccosy.CurvilinearCoordinateSystem.__setstate__(self, state[0])
        self.__dict__ = state[1]

    @property
    def reference(self) -> np.ndarray:
        """returns reference path used by CCosy due to slight modifications within the CCosy module"""
        return self._reference

    @property
    def ref_pos(self) -> np.ndarray:
        """position (s-coordinate) along reference path"""
        return self._ref_pos

    @property
    def ref_curv(self) -> np.ndarray:
        """curvature along reference path"""
        return self._ref_curv

    @property
    def ref_curv_d(self) -> np.ndarray:
        """curvature rate along reference path"""
        return self._ref_curv_d

    @property
    def ref_theta(self) -> np.ndarray:
        """orientation along reference path"""
        return self._ref_theta
