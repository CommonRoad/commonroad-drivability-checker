#pragma once
namespace collision {

enum DistanceSolverType { DIST_DEFAULT = 0, DIST_FCL = 1 };
enum DistanceRequestType { DRT_DISTANCE = 0, DRT_TOLERANCE_NEG = 1 };
enum DistanceNodeType { DNT_NARROWPHASE = 0, DNT_AABB = 1 };
class DistanceResult {
 public:
  DistanceResult(void) {
    m_min_distance = 0;
    m_tolerance_passed = false;
  }
  double getMinDistance(void) const { return m_min_distance; }
  double setMinDistance(double res) { return m_min_distance = res; }
  bool getTolerancePassed(void) const { return m_tolerance_passed; }
  bool setTolerancePassed(bool res) { return m_tolerance_passed = res; }

 protected:
  double m_min_distance;
  bool m_tolerance_passed;
};

class DistanceRequest {
 public:
  DistanceRequest(void) {
    dist_solver_type = DistanceSolverType::DIST_DEFAULT;
    computation_tolerance = 1e-6;
    compare_tolerance = 1e-6;
  }
  DistanceRequest(DistanceSolverType dist_solver_type) {
    this->dist_solver_type = dist_solver_type;
  }
  DistanceSolverType dist_solver_type;
  DistanceRequestType dist_request_type;
  DistanceNodeType dist_node_type;
  double computation_tolerance;
  double compare_tolerance;
};
}  // namespace collision
