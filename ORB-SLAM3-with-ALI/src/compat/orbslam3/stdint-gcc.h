// Stand-in for GCC's private <stdint-gcc.h>, which DBoW2's FORB.cpp and ORBmatcher.cc include unconditionally.
// MSVC has no such header; the standard <cstdint> provides everything they use.
#pragma once

#include <cstdint>
