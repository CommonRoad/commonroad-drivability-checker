#include "collision/application_settings.h"
#if ENABLE_SERIALIZER
#include <stdlib.h>
#include "collision/collision_object.h"
#include "collision/serialize/final/collision_object_export_final.h"
#include "collision/serialize/public/serialize_public.h"
#include "collision/serialize/serialize_reg_impl.h"

#include <s11n.net/s11n/algo.hpp>
#include <s11n.net/s11n/micro_api.hpp>
#include <s11n.net/s11n/proxy/std/vector.hpp>
#include <s11n.net/s11n/s11n_debuggering_macros.hpp>
#include <s11n.net/s11n/s11nlite.hpp>

namespace collision {
namespace serialize {
int serialize(const test::BroadphaseFailureObjObj &object,
              std::ostream &output_stream, const char *format) {
  BroadphaseFailure_obj_objExport bf_export(object);
  std::ios_base::fmtflags fmt_flags = output_stream.flags();
  std::streamsize old_precision = output_stream.precision();
  // std::hexfloat(output_stream); // hexadecimal float representation
  output_stream.precision(std::numeric_limits<double>::max_digits10 - 1);

  s11nlite::serializer_class(format);
  typedef s11nlite::micro_api<BroadphaseFailure_obj_objExport> BFExportAPI;
  BFExportAPI bf_export_api(format);
  if (bf_export_api.save(bf_export, output_stream)) {
    output_stream.precision(old_precision);
    output_stream.flags(fmt_flags);  // restore flags
    return 0;
  }
  output_stream.precision(old_precision);
  output_stream.flags(fmt_flags);  // restore flags
  return -1;
}
int serialize(const test::BroadphaseFailureCCObj &object,
              std::ostream &output_stream, const char *format) {
  BroadphaseFailure_cc_objExport bf_export(object);
  std::ios_base::fmtflags fmt_flags = output_stream.flags();
  std::streamsize old_precision = output_stream.precision();
  // std::hexfloat(output_stream); // hexadecimal float representation
  output_stream.precision(std::numeric_limits<double>::max_digits10 - 1);

  s11nlite::serializer_class(format);
  typedef s11nlite::micro_api<BroadphaseFailure_cc_objExport> BFExportAPI;
  BFExportAPI bf_export_api(format);
  if (bf_export_api.save(bf_export, output_stream)) {
    output_stream.precision(old_precision);
    output_stream.flags(fmt_flags);  // restore flags
    return 0;
  }
  output_stream.precision(old_precision);
  output_stream.flags(fmt_flags);  // restore flags
  return -1;
}
int serialize(const CollisionObject &collision_object,
              std::ostream &output_stream, const char *format) {
  std::ios_base::fmtflags fmt_flags = output_stream.flags();
  std::streamsize old_precision = output_stream.precision();
  // std::hexfloat(output_stream); // hexadecimal float representation
  output_stream.precision(std::numeric_limits<double>::max_digits10 - 1);
  ICollisionObjectExport_s11 *obj_export =
      static_cast<ICollisionObjectExport_s11 *>(collision_object.exportThis());
  if (!obj_export) {
    output_stream.precision(old_precision);
    output_stream.flags(fmt_flags);  // restore flags
    return -1;
  }
  std::shared_ptr<ICollisionObjectExport_s11> obj_export_ptr(obj_export);
  CollisionObjectExport_final_s11 obj_export_final(obj_export);
  s11nlite::serializer_class(format);
  typedef s11nlite::micro_api<CollisionObjectExport_final_s11> ColObjExportAPI;
  ColObjExportAPI obj_export_api(format);
  if (obj_export_api.save(obj_export_final, output_stream)) {
    output_stream.precision(old_precision);
    output_stream.flags(fmt_flags);  // restore flags
    return 0;
  }
  output_stream.precision(old_precision);
  output_stream.flags(fmt_flags);  // restore flags
  return -1;
}

int serialize(const CollisionChecker &collision_checker,
              std::ostream &output_stream, const char *format) {
  std::ios_base::fmtflags fmt_flags = output_stream.flags();
  std::streamsize old_precision = output_stream.precision();
  // std::hexfloat(output_stream); // hexadecimal float representation
  output_stream.precision(std::numeric_limits<double>::max_digits10 - 1);
  CollisionCheckerExport *cc_export =
      static_cast<CollisionCheckerExport *>(collision_checker.exportThis());
  if (!cc_export) {
    output_stream.precision(old_precision);
    output_stream.flags(fmt_flags);  // restore flags
    return -1;
  }
  std::shared_ptr<CollisionCheckerExport> cc_export_ptr(cc_export);
  s11nlite::serializer_class(format);
  typedef s11nlite::micro_api<CollisionCheckerExport> CCExportAPI;
  CCExportAPI obj_export_api(format);
  if (obj_export_api.save(*cc_export_ptr, output_stream)) {
    output_stream.precision(old_precision);
    output_stream.flags(fmt_flags);  // restore flags
    return 0;
  }
  output_stream.precision(old_precision);
  output_stream.flags(fmt_flags);  // restore flags
  return -1;
}

int deserialize(CollisionCheckerConstPtr &collision_checker,
                std::istream &input_stream, const char *format) {
  std::ios_base::fmtflags fmt_flags = input_stream.flags();
  // std::hexfloat(input_stream); // hexadecimal float representation
  std::streamsize old_precision = input_stream.precision();

  input_stream.precision(std::numeric_limits<double>::max_digits10 - 1);
  s11nlite::serializer_class(format);
  typedef s11nlite::micro_api<CollisionCheckerExport> CCImportAPI;
  CCImportAPI cc_import_api(format);
  std::shared_ptr<CollisionCheckerExport> loaded_obj_final_ptr(
      cc_import_api.load(input_stream));

  if (!loaded_obj_final_ptr.get()) {
    input_stream.precision(old_precision);
    input_stream.flags(fmt_flags);  // restore flags
    return 1;
  }
  collision::CollisionChecker *loaded_col_obj =
      loaded_obj_final_ptr.get()->loadObject();

  collision_checker = CollisionCheckerConstPtr(loaded_col_obj);
  input_stream.precision(old_precision);
  input_stream.flags(fmt_flags);  // restore flags
  if (!loaded_col_obj) {
    return 1;
  } else {
    return 0;
  }
}

int deserialize(test::BroadphaseFailureObjObj &bf_object,
                std::istream &input_stream, const char *format) {
  std::ios_base::fmtflags fmt_flags = input_stream.flags();
  // std::hexfloat(input_stream); // hexadecimal float representation
  std::streamsize old_precision = input_stream.precision();

  input_stream.precision(std::numeric_limits<double>::max_digits10 - 1);
  s11nlite::serializer_class(format);
  typedef s11nlite::micro_api<BroadphaseFailure_obj_objExport> BFImportAPI;
  BFImportAPI obj_import_api(format);
  std::shared_ptr<BroadphaseFailure_obj_objExport> loaded_obj_final_ptr(
      obj_import_api.load(input_stream));

  if (!loaded_obj_final_ptr.get() ||
      !loaded_obj_final_ptr.get()->getFailure()) {
    input_stream.precision(old_precision);
    input_stream.flags(fmt_flags);  // restore flags
    return -1;
  }

  bf_object = *(static_cast<const test::BroadphaseFailureObjObj *>(
      loaded_obj_final_ptr.get()->getFailure()));

  input_stream.precision(old_precision);
  input_stream.flags(fmt_flags);  // restore flags
  return 0;
}

int deserialize(test::BroadphaseFailureCCObj &bf_object,
                std::istream &input_stream, const char *format) {
  std::ios_base::fmtflags fmt_flags = input_stream.flags();
  // std::hexfloat(input_stream); // hexadecimal float representation
  std::streamsize old_precision = input_stream.precision();

  input_stream.precision(std::numeric_limits<double>::max_digits10 - 1);
  s11nlite::serializer_class(format);
  typedef s11nlite::micro_api<BroadphaseFailure_cc_objExport> BFImportAPI;
  BFImportAPI obj_import_api(format);
  std::shared_ptr<BroadphaseFailure_cc_objExport> loaded_obj_final_ptr(
      obj_import_api.load(input_stream));

  if (!loaded_obj_final_ptr.get() ||
      !loaded_obj_final_ptr.get()->getFailure()) {
    input_stream.precision(old_precision);
    input_stream.flags(fmt_flags);  // restore flags
    return -1;
  }

  bf_object = *(static_cast<const test::BroadphaseFailureCCObj *>(
      loaded_obj_final_ptr.get()->getFailure()));

  input_stream.precision(old_precision);
  input_stream.flags(fmt_flags);  // restore flags
  return 0;
}

int deserialize(CollisionObjectConstPtr &collision_object,
                std::istream &input_stream, const char *format) {
  std::ios_base::fmtflags fmt_flags = input_stream.flags();
  // std::hexfloat(input_stream); // hexadecimal float representation
  std::streamsize old_precision = input_stream.precision();

  input_stream.precision(std::numeric_limits<double>::max_digits10 - 1);
  s11nlite::serializer_class(format);
  typedef s11nlite::micro_api<CollisionObjectExport_final_s11> ColObjImportAPI;
  ColObjImportAPI obj_import_api(format);

  std::shared_ptr<CollisionObjectExport_final_s11> loaded_obj_final_ptr(
      obj_import_api.load(input_stream));

  if (!loaded_obj_final_ptr.get() || !loaded_obj_final_ptr.get()->base) {
    input_stream.precision(old_precision);
    input_stream.flags(fmt_flags);  // restore flags
    return -1;
  }
  std::shared_ptr<ICollisionObjectExport_s11> loaded_obj_ptr(
      loaded_obj_final_ptr.get()->base);
  collision::CollisionObject *loaded_col_obj = loaded_obj_ptr->loadObject();

  collision_object = CollisionObjectConstPtr(loaded_col_obj);

  if (!loaded_col_obj) {
    input_stream.precision(old_precision);
    input_stream.flags(fmt_flags);  // restore flags
    return 1;
  } else {
    input_stream.precision(old_precision);
    input_stream.flags(fmt_flags);  // restore flags
    return 0;
  }
}

}  // namespace serialize
}  // namespace collision
#endif
