
#ifndef CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_DECLARATIONS_H_
#define CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_DECLARATIONS_H_

#include <boost/align/aligned_allocator.hpp>
namespace collision {
template <typename T>
using aligned_vector =
    std::vector<T, boost::alignment::aligned_allocator<T, 16>>;
}

#endif /* CPP_COLLISION_INCLUDE_COLLISION_ACCELERATORS_DECLARATIONS_H_ */
