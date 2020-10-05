#pragma once

#include "collision/serialize/serialize.h"

#if ENABLE_SERIALIZER

#include <s11n.net/s11n/s11nlite.hpp>

#include "collision/serialize/collision_object_export_s11.h"
#include "collision/serialize/final/collision_object_export_final.h"

#include <s11n.net/s11n/io/serializers.hpp>  // utility code for s11n::io
#include <s11n.net/s11n/s11n_config.hpp>

#define S11N_TYPE collision::serialize::ICollisionObjectExport_s11
#define S11N_TYPE_NAME "ICollisionObjectExport_s11"
#define S11N_ABSTRACT_BASE
#include <s11n.net/s11n/reg_s11n_traits.hpp>
#undef S11N_ABSTRACT_BASE

#define S11N_TYPE collision::serialize::IShapeExport
#define S11N_TYPE_NAME "ShapeExport"
#define S11N_BASE_TYPE collision::serialize::ICollisionObjectExport_s11
#include <s11n.net/s11n/reg_s11n_traits.hpp>
#undef S11N_BASE_TYPE

#define S11N_TYPE collision::serialize::RectangleAABBExport
#define S11N_TYPE_NAME "RectangleAABBExport"
#define S11N_BASE_TYPE collision::serialize::ICollisionObjectExport_s11
#include <s11n.net/s11n/reg_s11n_traits.hpp>
#undef S11N_BASE_TYPE

#define S11N_TYPE collision::serialize::RectangleOBBExport
#define S11N_TYPE_NAME "RectangleOBBExport"
#define S11N_BASE_TYPE collision::serialize::ICollisionObjectExport_s11
#include <s11n.net/s11n/reg_s11n_traits.hpp>
#undef S11N_BASE_TYPE

#define S11N_TYPE collision::serialize::TriangleExport
#define S11N_TYPE_NAME "TriangleExport"
#define S11N_BASE_TYPE collision::serialize::ICollisionObjectExport_s11
#include <s11n.net/s11n/reg_s11n_traits.hpp>
#undef S11N_BASE_TYPE

#define S11N_TYPE collision::serialize::SphereExport
#define S11N_TYPE_NAME "SphereExport"
#define S11N_BASE_TYPE collision::serialize::ICollisionObjectExport_s11
#include <s11n.net/s11n/reg_s11n_traits.hpp>
#undef S11N_BASE_TYPE

#define S11N_TYPE collision::serialize::PointExport
#define S11N_TYPE_NAME "PointExport"
#define S11N_BASE_TYPE collision::serialize::ICollisionObjectExport_s11
#include <s11n.net/s11n/reg_s11n_traits.hpp>
#undef S11N_BASE_TYPE

#define S11N_TYPE collision::serialize::PolygonExport
#define S11N_TYPE_NAME "PolygonExport"
#define S11N_BASE_TYPE collision::serialize::ICollisionObjectExport_s11
#include <s11n.net/s11n/reg_s11n_traits.hpp>
#undef S11N_BASE_TYPE

#define S11N_TYPE collision::serialize::ShapeGroupExport
#define S11N_TYPE_NAME "ShapeGroupExport"
#define S11N_BASE_TYPE collision::serialize::ICollisionObjectExport_s11
#include <s11n.net/s11n/reg_s11n_traits.hpp>
#undef S11N_BASE_TYPE

#define S11N_TYPE collision::serialize::TimeVariantCollisionObjectExport
#define S11N_TYPE_NAME "TimeVariantCollisionObjectExport"
#define S11N_BASE_TYPE collision::serialize::ICollisionObjectExport_s11
#include <s11n.net/s11n/reg_s11n_traits.hpp>
#undef S11N_BASE_TYPE

#define S11N_TYPE collision::serialize::CollisionObjectExport_final_s11
#define S11N_TYPE_NAME "CollisionObjectExport_final_s11"
#include <s11n.net/s11n/reg_s11n_traits.hpp>

#define S11N_TYPE collision::serialize::CollisionCheckerExport
#define S11N_TYPE_NAME "CollisionCheckerExport"
#include <s11n.net/s11n/reg_s11n_traits.hpp>

#define S11N_TYPE collision::serialize::BroadphaseFailure_obj_objExport
#define S11N_TYPE_NAME "BroadphaseFailure_obj_objExport"
#include <s11n.net/s11n/reg_s11n_traits.hpp>

#define S11N_TYPE collision::serialize::BroadphaseFailure_cc_objExport
#define S11N_TYPE_NAME "BroadphaseFailure_cc_objExport"
#include <s11n.net/s11n/reg_s11n_traits.hpp>

#endif
