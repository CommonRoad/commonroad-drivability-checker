#pragma once

// warning: to be included only with a serialize_.cc file: unsafe streaming
// operator redefinition for a generic version see also:
// https://stackoverflow.com/questions/20786220/eigen-library-initialize-matrix-with-data-from-file-or-existing-stdvector

#include <Eigen/Dense>
#include <istream>

std::istream &operator>>(std::istream &in, Eigen::Vector2d &vec) {
  in >> vec[0];
  in >> vec[1];

  return in;
}

std::ostream &operator<<(std::ostream &out, const Eigen::Vector2d &vec) {
  out << vec[0];
  out << " ";
  out << vec[1];
  return out;
}

#define S11N_TYPE Eigen::Vector2d
#define S11N_TYPE_NAME "vector2d"
#define S11N_SERIALIZE_FUNCTOR s11n::streamable_type_serialization_proxy
#include <s11n.net/s11n/reg_s11n_traits.hpp>
