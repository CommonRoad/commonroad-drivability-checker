#include "collision/solvers/fcl/solver_entity_fcl.h"
#include "collision/collision_object_ex.h"
namespace collision {
namespace solvers {
namespace solverFCL {
void SolverEntity_FCLDeleter::operator()(SolverEntity_FCL *p) { delete p; }
}  // namespace solverFCL
}  // namespace solvers

}  // namespace collision
