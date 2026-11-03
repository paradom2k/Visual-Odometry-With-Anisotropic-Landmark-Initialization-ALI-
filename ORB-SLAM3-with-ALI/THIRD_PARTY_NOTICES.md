# Third-party notices

`ali-orbslam3` is a combined work that includes and links third-party open-source software.  The
project as a whole is distributed under the **GNU General Public License, version 3 or later**
(`LICENSE`), because it incorporates and links ORB-SLAM3, which is GPLv3.

This file is a notice and a compliance checklist, not legal advice.  For a commercial or closed-source
distribution, have counsel review the obligations and the DBoW2 note below.

## 1. Components in this repository

| Component | Version / snapshot | License | Location | Copyright |
| --- | --- | --- | --- | --- |
| ORB-SLAM3 | 1.0, source files dated 2022-02-10 | GPL-3.0-or-later | `thirdparty/orbslam3/` | © 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel, Juan D. Tardós (University of Zaragoza); © 2014-2016 Raúl Mur-Artal, José M. M. Montiel, Juan D. Tardós |
| DBoW2 | ORB-SLAM3's bundled copy | BSD-style, four conditions (see note) | `thirdparty/orbslam3/Thirdparty/DBoW2/` | © 2015 Dorian Galvez-Lopez |
| g2o | ORB-SLAM3's bundled copy | BSD-3-Clause | `thirdparty/orbslam3/Thirdparty/g2o/` | © 2011 Rainer Kuemmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige, Wolfram Burgard |
| Sophus | ORB-SLAM3's bundled copy | MIT | `thirdparty/orbslam3/Thirdparty/Sophus/` | © 2011-2017 Hauke Strasdat; © 2012-2017 Steven Lovegrove |

ORB-SLAM3 also embeds these permissively licensed pieces inside its own `src/` and `include/`:

| Embedded component | Origin | License |
| --- | --- | --- |
| `ORBextractor.cc` | modified from OpenCV `orb.cpp` | BSD |
| `PnPsolver.h`, `PnPsolver.cc` | modified EPnP by Vincent Lepetit | FreeBSD |
| `MLPnPsolver.h`, `MLPnPsolver.cc` | modified MLPnP by Steffen Urban (OpenGV) | BSD |
| `ORBmatcher::DescriptorDistance` | Stanford bit-twiddling hacks | public domain |
| `DBoW2/DUtils`, `DBoW2/DLib` | DLib/DUtils by Dorian Galvez-Lopez | BSD (covered by the DBoW2 license) |

These notices come from `thirdparty/orbslam3/Dependencies.md`; they are compatible with GPLv3 and are
preserved as part of the ORB-SLAM3 source distribution.

`thirdparty/orbslam3/` is an **unmodified upstream source subset**.  This project's changes are in
`patches/orbslam3/overlay/` and are applied to a build-tree copy by CMake.  Those changed files are
derived from ORB-SLAM3 and are therefore also GPLv3.  GPLv3 section 5(a) change notices are kept in the
files and in `patches/orbslam3/README.md`; a plain-language list is in `CHANGES.md`.

## 2. External components (not vendored)

| Component | Version used | License | How it is used |
| --- | --- | --- | --- |
| OpenCV | 4.6.0 | Apache-2.0 | image I/O, ORB, stereo matching, calibration |
| Eigen | 3.3.5 | MPL-2.0; some modules BSD-3-Clause | linear algebra throughout ORB-SLAM3 |
| GLFW | 3.x, matching library | zlib/libpng | `ali_ui` window and input |
| Windows SDK / MSVC runtime | build toolchain | Microsoft terms | Windows build and, for binary releases, the runtime DLLs |

None of these is copied into this repository, except the OpenCV runtime DLLs staged into `bin/` by the
build.  If you distribute a binary, include the applicable notices (OpenCV's Apache-2.0 text and its
bundled third-party notices, GLFW's zlib license) together with GPLv3's source requirement.

**Boost is not required.**  ORB-SLAM3 uses a handful of Boost.Serialization and Boost.Algorithm
headers for its atlas save/load path, which these configurations do not call.  `src/compat/orbslam3/boost/`
supplies no-op stand-ins so that the tree builds without a Boost installation.

## 3. DBoW2 licence note

DBoW2's licence is BSD-style but carries a fourth condition that applies to linking: if you use DBoW2
in a commercial product you are asked to contact the author.  It is compatible with GPLv3; the note is
reproduced here because it is easy to miss when reading only the first three clauses
(`thirdparty/orbslam3/Thirdparty/DBoW2/LICENSE.txt`).

## 4. Provenance of the Windows port

The Windows/MSVC adaptation layers — `src/compat/orbslam3/` (POSIX/Pangolin/OpenSSL/Boost stand-ins,
`jo_compat.h`, viewer stubs) and the MSVC fixes inside `patches/orbslam3/overlay/` — derive from the
sibling **GIO_GVIL** project by the same author, which is itself distributed under GPLv3.  They are
carried here under the same licence and the original notices are preserved in the files.

The ALI code (`TriangulateALI`, `Frame::mvKpCov`, `LocalMapping::SetALI` and the branch inside
`CreateNewMapPoints`) is original to this project and is GPLv3 like the rest of the overlay.
