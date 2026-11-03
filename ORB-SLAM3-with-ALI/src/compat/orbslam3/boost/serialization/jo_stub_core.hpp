#pragma once
// Common body of the Boost.Serialization stand-ins (rebuilt repo, round 72).
//
// All thirteen stub headers include this one: giving each of them the same definitions made the compiler
// report `access`/`make_array`/`serialize`/`nvp` as redefinitions as soon as a translation unit pulled in
// two of them.
#include <cstddef>
namespace boost {
namespace serialization {
struct access {
  template <class Archive, class T> static void serialize(Archive&, T&, unsigned int) {}
};
template <class T, std::size_t N> T* make_array(T (&array)[N]) { return array; }
template <class Archive, class T> void serialize(Archive&, T&, unsigned int) {}
}  // namespace serialization
namespace archive { namespace detail { struct nvp {}; } }
}  // namespace boost
