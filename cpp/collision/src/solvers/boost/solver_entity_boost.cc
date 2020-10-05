#include "collision/solvers/boost/solver_entity_boost.h"

#include "collision/collision_object_ex.h"
namespace collision {
namespace solvers {
namespace solverBoost {
void SolverEntity_BoostDeleter::operator()(SolverEntity_Boost *p) { delete p; }
}  // namespace solverBoost
}  // namespace solvers

}  // namespace collision
