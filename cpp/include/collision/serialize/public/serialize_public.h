#pragma once

#include <iostream>
#include "collision/collision_checker.h"
#include "collision/collision_object.h"

#include "collision/tests/broadphase_failure.h"

namespace collision {
namespace serialize {
int serialize(const test::BroadphaseFailureCCObj &bf_object,
              std::ostream &output_stream,
              const char *format = SERIALIZER_DEFAULT_FORMAT);
int deserialize(test::BroadphaseFailureCCObj &bf_object,
                std::istream &input_stream,
                const char *format = SERIALIZER_DEFAULT_FORMAT);
int serialize(const test::BroadphaseFailureObjObj &bf_object,
              std::ostream &output_stream,
              const char *format = SERIALIZER_DEFAULT_FORMAT);
int deserialize(test::BroadphaseFailureObjObj &bf_object,
                std::istream &input_stream,
                const char *format = SERIALIZER_DEFAULT_FORMAT);
int serialize(const CollisionObject &collision_object,
              std::ostream &output_stream,
              const char *format = SERIALIZER_DEFAULT_FORMAT);
int deserialize(CollisionObjectConstPtr &collision_object,
                std::istream &input_stream,
                const char *format = SERIALIZER_DEFAULT_FORMAT);
int serialize(const CollisionChecker &collision_object,
              std::ostream &output_stream,
              const char *format = SERIALIZER_DEFAULT_FORMAT);
int deserialize(CollisionCheckerConstPtr &collision_object,
                std::istream &input_stream,
                const char *format = SERIALIZER_DEFAULT_FORMAT);

}  // namespace serialize
}  // namespace collision
