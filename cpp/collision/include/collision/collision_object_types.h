
#pragma once

namespace collision {

enum CollisionObjectClass {
  OBJ_CLASS_UNKNOWN = 0,
  OBJ_CLASS_SHAPE = 1,
  OBJ_CLASS_SHAPEGROUP = 2,
  OBJ_CLASS_TVOBSTACLE = 3
};

constexpr int COL_OBJECT_TYPES_COUNT = 9;

enum CollisionObjectType {
  OBJ_TYPE_UNKNOWN = 0,
  OBJ_TYPE_AABB_BOX = 1,
  OBJ_TYPE_OBB_BOX = 2,
  OBJ_TYPE_SPHERE = 3,
  OBJ_TYPE_POINT = 4,
  OBJ_TYPE_TRIANGLE = 5,
  OBJ_TYPE_POLYGON = 6,
  OBJ_TYPE_SHAPEGROUP = 7,
  OBJ_TYPE_TVOBSTACLE = COL_OBJECT_TYPES_COUNT - 1
};

}  // namespace collision
