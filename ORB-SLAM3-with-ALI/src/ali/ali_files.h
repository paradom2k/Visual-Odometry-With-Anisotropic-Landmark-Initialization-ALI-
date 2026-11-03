#pragma once

// Filesystem helpers, deliberately Win32 rather than <filesystem>: the whole tree is built as C++14, which
// is the standard upstream ORB-SLAM3 is written to and one binary cannot mix standards across a library
// boundary without risking the standard-library ABI.

#include <string>
#include <vector>

namespace ali {

/// Every regular file in `directory`, sorted by name.
///
/// Sorted by NAME, not by the order the filesystem hands them over.  That is the whole time base for a
/// dataset dumped as one image per frame whose names are the timestamps: `1602075232933291776.png` sorts
/// into capture order as a string because the stamps are fixed-width nanoseconds, which is what makes the
/// 4Seasons extraction usable without a manifest.
bool listFiles(const std::string& directory, std::vector<std::string>* out, std::string* error);

/// True when the path names an existing directory.
bool isDirectory(const std::string& path);

/// True when the path names an existing regular file.
bool isFile(const std::string& path);

/// The image files of a directory, filtered by extension (`.png`, `.jpg`, `.jpeg`, `.pgm`, `.bmp`, `.tif`)
/// and sorted.  Returned as full paths.
bool listImages(const std::string& directory, std::vector<std::string>* out, std::string* error);

/// Every `.bag` in a directory, in PLAYBACK order.
///
/// Playback order is the bag's own index, not the alphabet's: these recordings are named
/// `2024-07-18-08-07-11_0_00.bag` through `_36_00.bag`, and sorting the names puts bag 10 before bag 2.
/// The index is the second-to-last underscore-separated number -- the last one is the camera -- and a name
/// that does not fit the pattern keeps its position by sorting last, rather than being silently reordered.
bool listBagFiles(const std::string& directory, std::vector<std::string>* out, std::string* error);

/// The numeric value of an image file name, as seconds.
///
/// The two shapes this project's datasets use, both handled: a name that is an integer number of
/// nanoseconds (`1602075232933291776.png`) and a name that is already seconds (`1602075232.933291776.png`
/// or `0000000001.000000000.png`).  Anything else returns 0, and the caller then falls back to the frame
/// index divided by the frame rate -- which is what a dataset with plain sequential names needs anyway.
double stampFromFileName(const std::string& path, bool* looked_like_time);

/// One line of a manifest: a stamp and an image path.
struct ManifestRow {
  double stamp = 0.0;
  std::string path;
};

/// Read a `<stamp> <path>` manifest.  Blank lines and `#` comments are skipped, and paths are resolved
/// against `root` when they are relative.
bool readManifest(const std::string& manifest, const std::string& root, std::vector<ManifestRow>* out,
                  std::string* error);

/// Where the running executable is.  Used to find `conf/` and `sol/` from a binary that may sit in
/// `<root>/build/src` or `<root>/bin`.
std::string executablePath();

/// Read every line of a text file, `#` comments and blank lines dropped.
bool readLines(const std::string& path, std::vector<std::string>* out, std::string* error);

}  // namespace ali
