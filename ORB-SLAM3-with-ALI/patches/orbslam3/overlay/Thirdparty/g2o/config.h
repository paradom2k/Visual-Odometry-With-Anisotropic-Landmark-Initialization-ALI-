// GIO_GVIL modification notice (2026-10-21).
// Derived from ORB-SLAM3 (GPLv3) and modified by GIO_GVIL; see patches/orbslam3/README.md.
// Derived mechanically from g2o's config.h.in (github_prj\20261011\g2o-20241228_git\config.h.in), with
// every #cmakedefine line expanded to its CMake-OFF form (the whole line is replaced, so the @VAR@ tails
// that the template carries for the ON case are dropped).  Not hand-authored.

#ifndef G2O_CONFIG_H
#define G2O_CONFIG_H

/* #undef G2O_HAVE_OPENGL */
/* #undef G2O_OPENGL_FOUND */
/* #undef G2O_OPENMP */
/* #undef G2O_SHARED_LIBS */
/* #undef G2O_LGPL_SHARED_LIBS */

// available sparse matrix libraries
/* #undef G2O_HAVE_CHOLMOD */
/* #undef G2O_HAVE_CSPARSE */

// logging framework available
/* #undef G2O_HAVE_LOGGING */

/* #undef G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS */

#ifdef G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS
#define G2O_DELETE_IMPLICITLY_OWNED_OBJECTS 0
#else
#define G2O_DELETE_IMPLICITLY_OWNED_OBJECTS 1
#endif

/* #undef G2O_CXX_COMPILER */
/* #undef G2O_SRC_DIR */

#endif

// required by this tree's g2o copy
#define G2O_FRAME_POINTER 0
