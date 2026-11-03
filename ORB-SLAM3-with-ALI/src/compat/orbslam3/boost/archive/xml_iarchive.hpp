#pragma once
// Class stand-in for xml_iarchive (rebuilt repo, round 72).  `System.cc`'s Atlas save/load path constructs these
// archives and streams into them; with JO_WITH_BOOST=OFF the streaming is deliberately a no-op, so the class
// only has to exist with the operators it is handed.  Naming every shape used by the two call sites keeps the
// member list honest instead of relying on a catch-all conversion.
#include <boost/serialization/jo_stub_core.hpp>
namespace boost { namespace archive {
class xml_iarchive {
 public:
  template <class Stream> explicit xml_iarchive(Stream&) {}
  template <class T> xml_iarchive& operator&(T&) { return *this; }
  template <class T> xml_iarchive& operator<<(const T&) { return *this; }
  template <class T> xml_iarchive& operator>>(T&) { return *this; }
  template <class T> void save_binary(const T&, std::size_t) {}
  void close() {}
};
}}  // namespace boost::archive
