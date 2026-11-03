// Stand-in for Pangolin (route E, rebuilt repo).  Nothing here draws: `Viewer.cc` and `MapDrawer.cc` are the
// only two files that include this header, and both are excluded from the route_e target, while the rest of
// the ported tree only needs the *names* that leak through it:
//
//   * `pangolin::OpenGlMatrix`, named by `include/MapDrawer.h`
//   * the global OpenGL scalar types, e.g. `GLubyte` in `include/Map.h` (`GLubyte* mThumbnail`)
//
// Upstream got the GL types from the platform GL headers that Pangolin included, i.e. in the *global*
// namespace -- so they are declared here at global scope, not inside `namespace pangolin`.
#pragma once

#include <cstddef>

typedef unsigned char GLubyte;
typedef unsigned int GLuint;
typedef int GLint;
typedef float GLfloat;
typedef double GLdouble;
typedef unsigned int GLenum;
typedef unsigned int GLbitfield;
typedef void GLvoid;

namespace pangolin {

/// The 4x4 OpenGL matrix the viewer/drawer would pass around; layout mirrors upstream (column-major, 16 floats).
struct OpenGlMatrix {
  OpenGlMatrix() {
    for (int i = 0; i < 16; ++i) {
      m[i] = 0.0f;
    }
    m[0] = m[5] = m[10] = m[15] = 1.0f;
  }

  static OpenGlMatrix Identity() { return OpenGlMatrix(); }

  void SetIdentity() {
    for (int i = 0; i < 16; ++i) {
      m[i] = 0.0f;
    }
    m[0] = m[5] = m[10] = m[15] = 1.0f;
  }

  float* operator[](int row) { return &m[row * 4]; }
  const float* operator[](int row) const { return &m[row * 4]; }

  float m[16];
};

}  // namespace pangolin
