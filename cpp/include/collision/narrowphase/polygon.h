#ifndef POLYGON_H_
#define POLYGON_H_

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

#include "collision/narrowphase/shape.h"
#include "collision/narrowphase/triangle.h"
#include "collision/plugins/triangulation/triangulate.h"
#include "collision/solvers/boost/boost_object_internal.h"

namespace collision {

class Polygon;

class ShapeGroup;

namespace solvers {
namespace solverBoost {
class BoostPolygon;
}
}

typedef std::shared_ptr<ShapeGroup> ShapeGroupPtr;
typedef std::shared_ptr<const ShapeGroup> ShapeGroupConstPtr;

typedef std::shared_ptr<Polygon> PolygonPtr;
typedef std::shared_ptr<const Polygon> PolygonConstPtr;

/*!
  \brief Polygon contains Triangles and Vertices

*/
class Polygon : public Shape {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Polygon(std::vector<Eigen::Vector2d> &vertices,
          std::vector<std::vector<Eigen::Vector2d>> &hole_vertices,
          std::vector<TriangleConstPtr> &mesh_triangles,
          const Eigen::Vector2d &_center = Eigen::Vector2d(0, 0));
#if ENABLE_TRIANGULATION
  Polygon(std::vector<Eigen::Vector2d> &vertices,
          std::vector<std::vector<Eigen::Vector2d>> &hole_vertices, int triangulation_method,
          triangulation::TriangulationQuality qual =
              triangulation::TriangulationQuality(),
          const Eigen::Vector2d &_center = Eigen::Vector2d(0, 0));
  /*Polygon(std::vector<Eigen::Vector2d> &vertices, int triangulation_method,
          triangulation::TriangulationQuality qual =
              triangulation::TriangulationQuality(),
          const Eigen::Vector2d &_center = Eigen::Vector2d(0, 0));
	*/
#endif

  Polygon(Polygon &&) = default;
  Polygon& operator=(Polygon&&) = default;
  Polygon& operator=(const Polygon&) = delete;

  Polygon(const Polygon &copy);

  Polygon *clone() const;

  ShapeType type(void) const;

  bool is_valid() const { return is_valid_;}

  bool rayTrace(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2,
                std::vector<LineSegment> &intersect) const override;

  virtual ~Polygon() {}

  bool isWithin(const Polygon &poly2) const;

  virtual void toString(std::ostringstream &stream) const;
  virtual void print(std::ostringstream &stream) const;
  virtual CollisionObjectConstPtr timeSlice(
      int time_idx, CollisionObjectConstPtr shared_ptr_this) const;

  ShapeGroupConstPtr getTrapezoids() const;

  std::vector<TriangleConstPtr> getTriangleMesh() const;
  std::vector<Eigen::Vector2d> getVertices() const;
  std::vector<std::vector<Eigen::Vector2d>> getHoleVertices() const;

  virtual CollisionObjectType getCollisionObjectType() const override {
    return CollisionObjectType::OBJ_TYPE_POLYGON;
  }

  solvers::solverBoost::BoostPolygon* getOrCreateBoostPolygon(void) const;

#if ENABLE_SERIALIZER
  serialize::ICollisionObjectExport *exportThis(void) const override;
#endif

 private:
  static constexpr ShapeType type_ = TYPE_POLYGON;
  fcl::CollisionGeometry<FCL_PRECISION> *createFCLCollisionGeometry(
      void) const override;
  fcl::CollisionObject<FCL_PRECISION> *createFCLCollisionObject(
      const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &)
      const override;

  mutable std::unique_ptr<solvers::solverBoost::BoostObjectInternal> boost_polygon_;
  mutable bool has_boost_polygon_ = false;
  std::vector<Eigen::Vector2d> vertices_;
  std::vector<std::vector<Eigen::Vector2d>> hole_vertices_;
  std::vector<TriangleConstPtr> mesh_triangles_;
  mutable bool is_valid_ = true;
};

}  // namespace collision

#endif
