#pragma once
namespace collision {

enum CollisionSolverType { COL_DEFAULT = 0, COL_FCL = 1 };

/*!
\brief Structure holding result for a collision request

*/
class CollisionResult {
 public:
  CollisionResult(void) { m_collides = false; }
  bool collides() const { return m_collides; }
  bool set_result(bool res) { return m_collides = res; }

 protected:
  bool m_collides;
};

/*!
\brief Universal structure specifying collision request properties

*/
class CollisionRequest {
 public:
  CollisionRequest(void) { col_solver_type = CollisionSolverType::COL_DEFAULT; }
  CollisionRequest(CollisionSolverType col_solver_type) {
    this->col_solver_type = col_solver_type;
  }
  CollisionSolverType col_solver_type;
};

}  // namespace collision
