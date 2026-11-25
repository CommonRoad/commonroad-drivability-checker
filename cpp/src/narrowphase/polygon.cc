#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include "collision/plugins/triangulation/triangulate.h"
#include "collision/raytrace_utils.h"
#include "collision/shape_group.h"
#include "collision/solvers/boost/boost_collision_queries.h"
#include "collision/solvers/fcl/fcl_decl.h"
#include "collision/solvers/fcl/fcl_transform.h"
#include "collision/solvers/accelerators/declarations.h"

#include "collision/narrowphase/polygon.h"

namespace {
	bool is_counter_clockwise(const std::vector<Eigen::Vector2d>& vertices) {
		double sum = 0.0;
		size_t n = vertices.size();

		for (size_t i = 0; i < n; i++) {
			const Eigen::Vector2d& p1 = vertices[i];
			const Eigen::Vector2d& p2 = vertices[(i + 1) % n];

			sum += (p2[0] - p1[0]) * (p2[1] + p1[1]);
		}

		return sum < 0;
	}
}

namespace collision {

/*!
 \brief Constructs a Polygon.
 \param[in] vertices - the outer ring vertices of the Polygon (counter-clockwise orientation). The outer ring is not required to be closed.
 \param[in] hole_vertices - the inner rings of vertices of the Polygon (counter-clockwise orientation), holes in the polygon. The inner rings
 are not required to be closed. The hole_verices are considered only for Polygon operations with Boost, such as Polygon enclosure, not for
 collision checking.
 \param[in] mesh_triangles - Triangles, used for collision checking
 \param[in] _center - center of the polygon. Please note that the vertice coordinates are to be specified in absolute coordinates,
 not relative to the center.
*/

Polygon::Polygon(std::vector<Eigen::Vector2d> &vertices,
                 std::vector<std::vector<Eigen::Vector2d>> &hole_vertices,
                 std::vector<TriangleConstPtr> &mesh_triangles,
                 const Eigen::Vector2d &_center)
    : Shape(_center) {
  vertices_ = vertices;
  hole_vertices_ = hole_vertices;
  mesh_triangles_ = mesh_triangles;
  validate_ring_ordering();
  invalidateCollisionEntityCache();
  compute_is_valid();
}
#if ENABLE_TRIANGULATION

/*!
 \brief Constructs a Polygon.
 \param[in] vertices - the outer ring vertices of the Polygon (counter-clockwise orientation). The outer ring is not required to be closed.
 \param[in] hole_vertices - the inner rings of vertices of the Polygon (counter-clockwise orientation), holes in the polygon. The inner rings
 are not required to be closed. The hole_verices are considered only for Polygon operations with Boost, such as Polygon enclosure, not for
 collision checking.
 \param[in] triangulation_method - library to be used for triangulation.
 \param[in] qual - required quality of triangulation
 \param[in] _center - center of the polygon. Please note that the vertice coordinates are to be specified in absolute coordinates,
 not relative to the center.
*/

Polygon::Polygon(std::vector<Eigen::Vector2d> &vertices,
                 std::vector<std::vector<Eigen::Vector2d>> &hole_vertices, int triangulation_method,
                 triangulation::TriangulationQuality qual,
                 const Eigen::Vector2d &_center)
    : Shape(_center) {
  hole_vertices_ = hole_vertices;
  vertices_ = vertices;
  validate_ring_ordering();
  if (qual.bb_only) {
    triangulation::do_triangulate_aabb(vertices, mesh_triangles_);
  } else {
    if (!qual.use_quality) {
      triangulation::do_triangulate(vertices, mesh_triangles_, triangulation_method);
    } else {
      triangulation::do_triangulateQuality(vertices, mesh_triangles_, triangulation_method, qual);
    }
  }
  invalidateCollisionEntityCache();
  compute_is_valid();
}

#endif

void Polygon::compute_is_valid() {
  for (auto &tr : mesh_triangles_) {
	  if(tr->is_valid()) {
		  is_valid_ = true;
		  return;
	  }
  }
  is_valid_ = false;
}

void Polygon::validate_ring_ordering() {
	if (!is_counter_clockwise(vertices_)) {
		std::reverse(vertices_.begin(), vertices_.end());
	}
	for (auto& inner_ring: hole_vertices_) {
		if (!is_counter_clockwise(inner_ring)) {
			std::reverse(inner_ring.begin(), inner_ring.end());
		}
	}
}

/*!
 \brief A helper function that is called from the rayTracePrimitive function.
 Given the query line segment [point1, point2], it outputs the part(s) of the line segment that intersect with the Polygon,
 taking into the account only the mesh_triangles, not hole_vertices.
 \param[in] point1 - start of the query line segment
 \param[in] point2 - end of the query line segment
 \param[out] intersect - vector to which the output line segments are to be appended
*/

bool Polygon::rayTrace(const Eigen::Vector2d &point1,
                       const Eigen::Vector2d &point2,
                       std::vector<LineSegment> &intersect) const {
  LineSegment obj_segment(point1, point2);
  bool res = false;
  for (auto &tri : mesh_triangles_) {
    std::vector<Eigen::Vector2d> inters1;
    for (auto &segm : tri->segments()) {
      res = segm.intersect(obj_segment, inters1) || res;
    }
    collision::raytrace::rayTracePostprocess(point1, point2, inters1, intersect,
                                             tri.get());
  }
  return res;
}

/*!
 \brief Clones the Polygon
*/

Polygon *Polygon::clone() const { return new Polygon(*this); }

/*!
 \brief Copy constructor for a Polygon
*/

Polygon::Polygon(const Polygon &copy) : Shape(copy) {
  vertices_ = copy.getVertices();
  hole_vertices_ = copy.getHoleVertices();
  mesh_triangles_ = copy.getTriangleMesh();
  is_valid_ = copy.is_valid();
  invalidateCollisionEntityCache();
}

/*!
 \brief Returns the type of the Shape
*/

ShapeType Polygon::type(void) const { return type_; }

/*!
 \brief does nothing
*/

void Polygon::print(std::ostringstream &stream) const {}

/*!
 \brief Prints out important information about the Polygon
 \param[out] stream - output stringstream to print the information to
*/

void Polygon::toString(std::ostringstream &stream) const {
  stream << "Polygon "
         << "triangles: ";
  for (auto &tri : mesh_triangles_) {
    tri->print(stream);
  }
  stream << "\\Polygon " << std::endl;
}

/*!
 \brief Checks if the Polygon is fully contained within another Polygon.
 \param[in] poly2 - the other polygon
*/

bool Polygon::isWithin(const Polygon &poly2) const {
  using namespace collision::solvers::solverBoost;
  const BoostPolygon *this_boost =
      getOrCreateBoostPolygon();
  const BoostPolygon *other_boost = poly2.getOrCreateBoostPolygon();
  if (!this_boost || !other_boost) {
    throw 0;
  }
  return boost_within(*this_boost, *other_boost);
}

/*!
 \brief creates FCL collision geometry for a Polygon. This is a FCL library internal representation used for collision checking.
 Invalid triangles are not added. If the polygon has no triangles added, it is considered to be invalid.
 All collision functions will return false for an invalid Polygon
*/

fcl::CollisionGeometry<FCL_PRECISION> *Polygon::createFCLCollisionGeometry(
    void) const {

  // exclude invalid triangles from the fcl model
  aligned_vector<const Triangle*> tris;
  tris.reserve(mesh_triangles_.size());
  for (auto &tr : mesh_triangles_) {
	  if(tr->is_valid()) {
		  tris.push_back(tr.get());
	  }
  }
  if (tris.size()) {
	  fcl::BVHModel<fcl::AABB<FCL_PRECISION>> *model =
	      new fcl::BVHModel<fcl::AABB<FCL_PRECISION>>();
	  model->beginModel(tris.size(), tris.size() * 3);
	  for (auto tr : tris) {
		Eigen::Vector2d v2d = tr->v1();
		fcl::Vector3<FCL_PRECISION> v1(v2d[0], v2d[1], 0);
		v2d = tr->v2();
		fcl::Vector3<FCL_PRECISION> v2(v2d[0], v2d[1], 0);
		v2d = tr->v3();
		fcl::Vector3<FCL_PRECISION> v3(v2d[0], v2d[1], 0);
		model->addTriangle(v1, v2, v3);
	  }
	  model->endModel();
	  is_valid_ = true;
	  return model;
  } else {
	  is_valid_ = false;
	  fcl::BVHModel<fcl::AABB<FCL_PRECISION>> *model =
	  	      new fcl::BVHModel<fcl::AABB<FCL_PRECISION>>();
	  return model;
	  // will crash upon collision detection with the created fcl collision object unless is_valid_ is checked
  }
}

/*!
 \brief creates FCL collision object for a Polygon. This is a FCL library internal representation used for collision checking.
 \param[in] col_geom - corresponding FCL collision geometry
*/

fcl::CollisionObject<FCL_PRECISION> *Polygon::createFCLCollisionObject(
    const std::shared_ptr<fcl::CollisionGeometry<FCL_PRECISION>> &col_geom)
    const {
  return new fcl::CollisionObject<FCL_PRECISION>(col_geom);
}

RectangleAABBConstPtr Polygon::getAABB() const {
  if (!is_valid_) {
	return std::make_shared<const RectangleAABB>(0, 0);
  } else {
	return CollisionObject::getAABB();
  }
}

/*!
 \brief a getter for mesh_triangles_
*/

std::vector<TriangleConstPtr> Polygon::getTriangleMesh() const {
  return mesh_triangles_;
}

/*!
 \brief a getter for vertices_
*/

std::vector<Eigen::Vector2d> Polygon::getVertices() const { return vertices_; }


/*!
 \brief a getter for hole_vertices_
*/

std::vector<std::vector<Eigen::Vector2d>> Polygon::getHoleVertices() const {
  return hole_vertices_;
}

/*!
 \brief Creates BoostPolygon object from the Polygon or returns the stored boost_polygon_.
 BoostPolygon enables the use of boost::geometry library functions.
*/

BoostPolygon* Polygon::getOrCreateBoostPolygon(void) const {
	if (!has_boost_polygon_) {
		boost_polygon_.reset(new BoostPolygon(this));
		has_boost_polygon_ = true;
	}
	return static_cast<BoostPolygon*>(boost_polygon_.get());
}

#if ENABLE_SERIALIZER

namespace serialize {
ICollisionObjectExport *exportObject(const collision::Polygon &);
}

/*!
 \brief Exports the Polygon into a serializable object. S11n library is used for serialization.
*/

serialize::ICollisionObjectExport *Polygon::exportThis(void) const {
  return serialize::exportObject(*this);
}
#endif

}  // namespace collision
