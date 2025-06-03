#pragma once

#include "collision/application_settings.h"

#if ENABLE_SERIALIZER

#include "collision/i_collision_object_export.h"


#include "collision/serialize/shape_export.h"

#include "collision/serialize/rectangle_aabb_export.h"

#include "collision/serialize/rectangle_obb_export.h"

#include "collision/serialize/triangle_export.h"

#include "collision/serialize/sphere_export.h"

#include "collision/serialize/point_export.h"

#include "collision/serialize/polygon_export.h"

#include "collision/serialize/tv_object_export.h"

#include "collision/serialize/shape_group_export.h"

#include "collision/serialize/collision_checker_export.h"

#include "collision/serialize/broadphase_failure_export.h"

#include "collision/serialize/broadphase_failure_obj_obj_export.h"

#include "collision/serialize/broadphase_failure_cc_obj_export.h"

#include <Eigen/Dense>

#endif
