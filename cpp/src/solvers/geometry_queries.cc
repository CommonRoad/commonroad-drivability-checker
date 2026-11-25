#include <fcl/math/bv/OBB.h>
#include "collision/narrowphase/polygon.h"
#include "collision/narrowphase/rectangle_obb.h"
#include "collision/narrowphase/sphere.h"
#include "collision/shape_group.h"
#include "collision/solvers/boost/boost_collision_queries.h"

#include "collision/solvers/geometry_queries.h"
#include <boost/range/combine.hpp>

namespace collision {

using namespace solvers::solverBoost;

namespace detail {
namespace geometry_queries {

/*!
 \brief Creates fcl::OBB from RectangleOBB
 \param[in] obb - input oriented rectangle
 \param[out] fcl_obb - output fcl::OBB
*/

int fillFclOBBHelper(const RectangleOBB* obb,
                     fcl::OBB<FCL_PRECISION>* fcl_obb) {
  fcl_obb->axis = fcl::Matrix3<FCL_PRECISION>::Zero();
  fcl_obb->To = fcl::Vector3<FCL_PRECISION>::Zero();
  fcl_obb->extent = fcl::Vector3<FCL_PRECISION>::Zero();

  fcl_obb->To.topRows(2) = obb->center().cast<double>();

  if (obb->r_x() > obb->r_y())

  {
    fcl_obb->axis.block<2, 1>(0, 0) = obb->local_x_axis();
    fcl_obb->axis.block<2, 1>(0, 1) = obb->local_y_axis();
    fcl_obb->axis(2, 2) = 1;

    fcl_obb->extent(0) = obb->r_x();
    fcl_obb->extent(1) = obb->r_y();

  } else {
    fcl_obb->axis.block<2, 1>(0, 0) = obb->local_y_axis();
    fcl_obb->axis.block<2, 1>(0, 1) = obb->local_x_axis();
    fcl_obb->axis(2, 2) = 1;
    fcl_obb->extent(0) = obb->r_x();
    fcl_obb->extent(1) = obb->r_y();
  }

  return 0;
}

/*!
 \brief Creates fcl::OBB from OBB
 \param[in] obb - input oriented rectangle
 \param[out] fcl_obb - output fcl::OBB
*/

int fillFclOBBHelper(const OBB* obb, fcl::OBB<FCL_PRECISION>* fcl_obb) {
  fcl_obb->axis = fcl::Matrix3<FCL_PRECISION>::Zero();
  fcl_obb->To = fcl::Vector3<FCL_PRECISION>::Zero();
  fcl_obb->extent = fcl::Vector3<FCL_PRECISION>::Zero();
  fcl_obb->To.topRows(2) = obb->center().cast<double>();

  if (obb->r_x() > obb->r_y())

  {
    fcl_obb->axis.block<2, 1>(0, 0) = obb->local_x_axis();
    fcl_obb->axis.block<2, 1>(0, 1) = obb->local_y_axis();
    fcl_obb->axis(2, 2) = 1;

    fcl_obb->extent(0) = obb->r_x();
    fcl_obb->extent(1) = obb->r_y();

  } else {
    fcl_obb->axis.block<2, 1>(0, 0) = obb->local_y_axis();
    fcl_obb->axis.block<2, 1>(0, 1) = obb->local_x_axis();
    fcl_obb->axis(2, 2) = 1;
    fcl_obb->extent(0) = obb->r_x();
    fcl_obb->extent(1) = obb->r_y();
  }

  return 0;
}

// todo: add AABB handling

/*!
 \brief Creates oriented bounding box that encloses two input oriented bounding boxes
 \param[in] first - first oriented rectangle
 \param[in] second - second oriented rectangle
*/

RectangleOBBConstPtr ccd_merge_entities(const RectangleOBB* first,
                                        const RectangleOBB* second) {
  fcl::OBB<FCL_PRECISION> this_obb;
  fcl::OBB<FCL_PRECISION> other_obb;

  fillFclOBBHelper(first, &this_obb);
  fillFclOBBHelper(second, &other_obb);

  fcl::OBB<FCL_PRECISION> merged_obb = this_obb + other_obb;

  Eigen::Matrix2d local_ax;

  local_ax = merged_obb.axis.block<2, 2>(0, 0).cast<double>();
  double rx = merged_obb.extent(0);
  double ry = merged_obb.extent(1);
  Eigen::Vector2d cent = merged_obb.To.topRows(2).cast<double>();

  RectangleOBBConstPtr ret(new collision::RectangleOBB(rx, ry, local_ax, cent));

  return (ret);
}

/*!
 \brief Merges two ShapeGroups by merging their shapes pairwise;
 if one ShapeGroup has more shapes than the other one, their additional shapes are also included in the result.
 \param[in] first - first ShapeGroup
 \param[in] second - second ShapeGroup
*/

ShapeGroupConstPtr ccd_merge_entities(const ShapeGroup* first,
                                        const ShapeGroup* second) {

  std::vector<ShapeConstPtr> shapes1 = first->unpack();
  std::vector<ShapeConstPtr> shapes2 = second->unpack();

  unsigned int maxSize = std::max(shapes1.size(), shapes2.size());

  ShapeGroup *result = new ShapeGroup();

  for (unsigned int i = 0; i < maxSize; i++) {
    ShapeConstPtr s1 = nullptr;
    ShapeConstPtr s2 = nullptr;

    if (i < shapes1.size()) {
      s1 = shapes1[i];
    }
    if (i < shapes2.size()) {
      s2 = shapes2[i];
    }

    if (s1 && s2) {
      CollisionObjectConstPtr merged = collision::geometry_queries::ccd_merge_entities(s1.get(), s2.get());
      result->addToGroup(std::static_pointer_cast<const Shape>(merged));
    }
    else if (s1) {
      result->addToGroup(s1);
    }
    else if (s2) {
      result->addToGroup(s2);
    }
  }

  return ShapeGroupConstPtr(result);
}

/*!
 \brief Compares two double numbers
 Returns true if they are within epsilon.
 \param[in] A - first number
 \param[in] B - second number
 \param[in] epsilon - comparison precision
*/

bool cmpd(double A, double B, double epsilon = 1e-7)
{
    return (fabs(A - B) < epsilon);
}

/*!
 \brief Creates an oriented rectangle that lies between the centers of two spheres. Its radius is the radius of the two spheres.
 The spheres must have the same radius.
 \param[in] first - first Sphere
 \param[in] second - second Sphere
 \param[out] ret - ShapeGroup to which the resulting oriented rectangle is appended
*/

int ccd_merge_entities(const Sphere* first,
                                        const Sphere* second, ShapeGroupPtr ret)
{
	if(!cmpd(first->radius(),second->radius(), 1e-7))
	{
		return -1;
	}
	ret->addToGroup(std::static_pointer_cast<const collision::Shape>(first->shared_from_this()));
	ret->addToGroup(std::static_pointer_cast<const collision::Shape>(second->shared_from_this()));

	auto rect=collision::geometry_queries::create_rectangle_obb_from_points(first->center(), second->center(),first->radius()*2);
	ret->addToGroup(rect);

	return 0;
}

/*!
 \brief Creates oriented bounding box that encloses two input oriented bounding boxes
 \param[in] first - first oriented rectangle
 \param[in] second - second oriented rectangle
*/

OBB merge_obbs(const OBB& first, const OBB& second) {
  OBB obb_merged;

  fcl::OBB<FCL_PRECISION> this_obb;
  fcl::OBB<FCL_PRECISION> other_obb;

  fillFclOBBHelper(&first, &this_obb);
  fillFclOBBHelper(&second, &other_obb);

  fcl::OBB<FCL_PRECISION> merged_obb = this_obb + other_obb;

  Eigen::Matrix2d local_ax;

  local_ax = merged_obb.axis.block<2, 2>(0, 0).cast<double>();
  Eigen::Vector2d r(merged_obb.extent(0), merged_obb.extent(1));

  Eigen::Vector2d cent = merged_obb.To.topRows(2).cast<double>();

  return OBB(local_ax, r, cent);
}

/*!
 \brief Creates oriented bounding box from input axis-aligned bounding box
 \param[in] aabb - input axis-aligned bounding box
*/

inline RectangleOBBConstPtr obb_from_aabb(const RectangleAABB* aabb) {
  return std::make_shared<const RectangleOBB>(aabb->r_x(), aabb->r_y(), 0,
                                              aabb->center());
}

}  // namespace geometry_queries
}  // namespace detail


namespace geometry_queries {

/*!
 \brief Merges two objects through calling the corresponding function above.
 These can be two boxes (oriented or axis-aligned), two spheres or two ShapeGroups containing the previously mentioned types of objects.
 \param[in] first - first object
 \param[in] second - second object
*/

CollisionObjectConstPtr ccd_merge_entities(const CollisionObject* first,
                                           const CollisionObject* second) {
  if ((first->getCollisionObjectType() == OBJ_TYPE_AABB_BOX) ||
      (second->getCollisionObjectType() == OBJ_TYPE_AABB_BOX)) {
    RectangleOBBConstPtr old_rect1;
    RectangleOBBConstPtr old_rect2;
    if (first->getCollisionObjectType() == OBJ_TYPE_AABB_BOX) {
      old_rect1 = detail::geometry_queries::obb_from_aabb(
          static_cast<const RectangleAABB*>(first));
      first = old_rect1.get();
    }
    if (second->getCollisionObjectType() == OBJ_TYPE_AABB_BOX) {
      old_rect2 = detail::geometry_queries::obb_from_aabb(
          static_cast<const RectangleAABB*>(second));
      second = old_rect2.get();
    }
    return detail::geometry_queries::ccd_merge_entities(
        static_cast<const RectangleOBB*>(first),
        static_cast<const RectangleOBB*>(second));
  }

  if (first->getCollisionObjectType() == OBJ_TYPE_OBB_BOX &&
      second->getCollisionObjectType() == OBJ_TYPE_OBB_BOX) {
    return detail::geometry_queries::ccd_merge_entities(
        static_cast<const RectangleOBB*>(first),
        static_cast<const RectangleOBB*>(second));
  }

  if (first->getCollisionObjectType() == OBJ_TYPE_SHAPEGROUP &&
      second->getCollisionObjectType() == OBJ_TYPE_SHAPEGROUP) {
        return detail::geometry_queries::ccd_merge_entities(
          static_cast<const ShapeGroup*>(first),
          static_cast<const ShapeGroup*>(second));
  }
  if(first->getCollisionObjectType()==OBJ_TYPE_SPHERE &&
       second->getCollisionObjectType()==OBJ_TYPE_SPHERE)
       {
	  	  ShapeGroupPtr sg_merge_res(new ShapeGroup());
	  	  int ret_merge=detail::geometry_queries::ccd_merge_entities(
	  	        static_cast<const Sphere*>(first),
	  	        static_cast<const Sphere*>(second),sg_merge_res);
	  	  if(ret_merge)
	  		  return nullptr;
	  	  else
	  		  return sg_merge_res;
       }
  /*
            else if(first->getCollisionObjectType()==OBJ_TYPE_POLYGON &&
     second->getCollisionObjectType()==OBJ_TYPE_POLYGON)
            {
                    PolygonConstPtr new_poly;
                    int res=boost_poly_poly_convex_hull(*(static_cast<const
     Polygon*>(first.get())),*(static_cast<const
     Polygon*>(second.get())),new_poly); if(res) return nullptr; return
     new_poly;
            }
   */
  else {
      std::cout << "wrong type" << std::endl;
      return nullptr;
  }
}

/*!
 \brief Checks if the input oriented rectangle is completely within the given Polygons
 \param[in] sg - ShapeGroup with input Polygons
 \param[in] obb - input oriented rectangle
 \param[out] res - result
*/

std::size_t test_polygon_enclosure(const ShapeGroup& sg,
                                   const RectangleOBB& obb, bool& res) {
  if (boost_polygon_enclosure(sg, obb, res)) {
    return 1;
  } else
    return 0;
}

/*!
 \brief Creates an oriented rectangle with its center line lying between the given two points with given width
 \param[in] pt1 - first point
 \param[in] pt2 - second point
 \param[in] rect_width - required width of the oriented rectangle
*/

RectangleOBBConstPtr create_rectangle_obb_from_points(Eigen::Vector2d pt1,
                                                      Eigen::Vector2d pt2,
                                                      double rect_width) {
  Eigen::Matrix2d local_axes;

  Eigen::Vector2d v = pt2 - pt1;

  Eigen::Vector2d center = (pt1 + pt2) / 2;

  double length = v.norm();

  if (length < 1e-10) return 0;

  v = v / length;

  local_axes.col(0) = v;
  local_axes.col(1) = Eigen::Vector2d(-v(1), v(0));
  return std::make_shared<const RectangleOBB>(length / 2, rect_width / 2,
                                              local_axes, center);
}

/*!
 \brief Creates oriented rectangles that represent the given list of vertices.
 It is useful for e.g. creating oriented rectangles for the road boundary.
 \param[in] verts - vector of input 2D vertices
 \param[out] sg_rects_ptr - ShapeGroup to which the output oriented rectangles are appended to
 \param[in] rect_width - required width of oriented rectangles
*/

int create_rectangles_obb_from_vertex_list(std::vector<Eigen::Vector2d>& verts,
                                           collision::ShapeGroup* sg_rects_ptr,
                                           double rect_width) {
  for (auto vert = verts.begin(); vert < verts.end() - 1; vert++) {
    collision::RectangleOBBConstPtr rect_new =
        create_rectangle_obb_from_points(*vert, *(vert + 1), rect_width);
    if (rect_new) sg_rects_ptr->addToGroup(rect_new);
  }
  if (verts.size() > 1) {
    collision::RectangleOBBConstPtr rect_new = create_rectangle_obb_from_points(
        *(verts.end() - 1), *(verts.begin()), rect_width);
    if (rect_new) sg_rects_ptr->addToGroup(rect_new);
  }
  return 0;
}

}  // namespace geometry_queries
}  // namespace collision
