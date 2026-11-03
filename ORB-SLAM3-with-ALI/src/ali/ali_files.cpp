#include "ali_files.h"

#include <windows.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace ali {
namespace {

std::string lower(std::string text) {
  for (char& c : text) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return text;
}

std::string trim(const std::string& text) {
  const char* spaces = " \t\r\n";
  const std::size_t first = text.find_first_not_of(spaces);
  if (first == std::string::npos) {
    return std::string();
  }
  return text.substr(first, text.find_last_not_of(spaces) - first + 1);
}

bool hasImageExtension(const std::string& name) {
  const std::string v = lower(name);
  const char* extensions[] = {".png", ".jpg", ".jpeg", ".pgm", ".bmp", ".tif", ".tiff"};
  for (const char* ext : extensions) {
    const std::size_t n = std::strlen(ext);
    if (v.size() > n && v.compare(v.size() - n, n, ext) == 0) {
      return true;
    }
  }
  return false;
}

/// The name without its directory and without its extension: an image file's identity.
std::string stem(const std::string& path) {
  std::string name = path;
  const std::size_t slash = name.find_last_of("/\\");
  if (slash != std::string::npos) {
    name = name.substr(slash + 1);
  }
  const std::size_t dot = name.find_last_of('.');
  if (dot != std::string::npos) {
    name = name.substr(0, dot);
  }
  return name;
}

}  // namespace

bool listFiles(const std::string& directory, std::vector<std::string>* out, std::string* error) {
  out->clear();
  if (directory.empty()) {
    if (error != nullptr) {
      *error = "no directory given";
    }
    return false;
  }
  WIN32_FIND_DATAA data;
  const std::string pattern = directory + "\\*";
  HANDLE handle = FindFirstFileA(pattern.c_str(), &data);
  if (handle == INVALID_HANDLE_VALUE) {
    if (error != nullptr) {
      *error = "cannot list " + directory + " (error " + std::to_string(GetLastError()) + ")";
    }
    return false;
  }
  do {
    if ((data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0) {
      continue;
    }
    out->push_back(directory + "/" + data.cFileName);
  } while (FindNextFileA(handle, &data) != 0);
  FindClose(handle);
  std::sort(out->begin(), out->end());
  if (out->empty()) {
    if (error != nullptr) {
      *error = directory + " holds no files";
    }
    return false;
  }
  return true;
}

bool isDirectory(const std::string& path) {
  const DWORD attributes = GetFileAttributesA(path.c_str());
  return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
}

bool isFile(const std::string& path) {
  const DWORD attributes = GetFileAttributesA(path.c_str());
  return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
}

bool listImages(const std::string& directory, std::vector<std::string>* out, std::string* error) {
  std::vector<std::string> all;
  if (!listFiles(directory, &all, error)) {
    return false;
  }
  out->clear();
  for (const std::string& file : all) {
    if (hasImageExtension(file)) {
      out->push_back(file);
    }
  }
  if (out->empty()) {
    if (error != nullptr) {
      *error = directory + " holds no image files";
    }
    return false;
  }
  return true;
}

/// The playback index of a bag named `<date>_<bag>_<camera>.bag`, or -1 if the name is something else.
static int bagIndex(const std::string& path) {
  std::string name = stem(path);          // without directory and extension
  const std::size_t last = name.rfind('_');
  if (last == std::string::npos || last == 0) {
    return -1;
  }
  const std::size_t prev = name.rfind('_', last - 1);
  if (prev == std::string::npos) {
    return -1;
  }
  const std::string middle = name.substr(prev + 1, last - prev - 1);
  if (middle.empty()) {
    return -1;
  }
  for (std::size_t i = 0; i < middle.size(); ++i) {
    if (middle[i] < '0' || middle[i] > '9') {
      return -1;
    }
  }
  return std::atoi(middle.c_str());
}

bool listBagFiles(const std::string& directory, std::vector<std::string>* out, std::string* error) {
  std::vector<std::string> all;
  if (!listFiles(directory, &all, error)) {
    return false;
  }
  out->clear();
  for (const std::string& file : all) {
    if (file.size() > 4 && file.compare(file.size() - 4, 4, ".bag") == 0) {
      out->push_back(file);
    }
  }
  if (out->empty()) {
    if (error != nullptr) {
      *error = directory + " holds no .bag files";
    }
    return false;
  }
  // Sorted by the bag's own index; a name that does not carry one keeps its alphabetical place at the end
  // rather than being dropped, because a directory holding both bag sets is a legitimate thing to point at.
  std::stable_sort(out->begin(), out->end(), [](const std::string& a, const std::string& b) {
    const int ia = bagIndex(a);
    const int ib = bagIndex(b);
    if (ia < 0 && ib < 0) return a < b;
    if (ia < 0) return false;
    if (ib < 0) return true;
    if (ia != ib) return ia < ib;
    return a < b;
  });
  return true;
}

double stampFromFileName(const std::string& path, bool* looked_like_time) {
  const std::string name = stem(path);
  if (looked_like_time != nullptr) {
    *looked_like_time = false;
  }
  // A name that is not all digits and at most one dot is not a timestamp.  The two accepted shapes:
  // nanoseconds as an integer, or seconds as a decimal.  The discriminator is the magnitude: a value above
  // 1e12 cannot be seconds (that would be the year 33658), so it is nanoseconds.
  bool digits = !name.empty();
  int dots = 0;
  for (char c : name) {
    if (c == '.') {
      ++dots;
    } else if (!std::isdigit(static_cast<unsigned char>(c))) {
      digits = false;
      break;
    }
  }
  if (!digits || dots > 1) {
    return 0.0;
  }
  const double value = std::atof(name.c_str());
  if (looked_like_time != nullptr) {
    *looked_like_time = true;
  }
  return value > 1.0e12 ? value * 1.0e-9 : value;
}

bool readManifest(const std::string& manifest, const std::string& root, std::vector<ManifestRow>* out,
                  std::string* error) {
  std::ifstream in(manifest);
  if (!in) {
    if (error != nullptr) {
      *error = "cannot open manifest " + manifest;
    }
    return false;
  }
  out->clear();
  std::string line;
  while (std::getline(in, line)) {
    const std::string text = trim(line);
    if (text.empty() || text[0] == '#') {
      continue;
    }
    std::istringstream stream(text);
    ManifestRow row;
    if (!(stream >> row.stamp)) {
      continue;
    }
    std::string rest;
    std::getline(stream, rest);
    rest = trim(rest);
    if (rest.empty()) {
      continue;
    }
    const bool absolute = (rest.size() > 1 && rest[1] == ':') || rest[0] == '/' || rest[0] == '\\';
    row.path = (absolute || root.empty()) ? rest : root + "/" + rest;
    out->push_back(row);
  }
  if (out->empty()) {
    if (error != nullptr) {
      *error = "manifest " + manifest + " holds no usable rows";
    }
    return false;
  }
  return true;
}

std::string executablePath() {
  char buffer[MAX_PATH * 4];
  const DWORD written = GetModuleFileNameA(nullptr, buffer, sizeof(buffer));
  if (written == 0 || written == sizeof(buffer)) {
    return std::string();
  }
  return std::string(buffer, written);
}

bool readLines(const std::string& path, std::vector<std::string>* out, std::string* error) {
  std::ifstream in(path);
  if (!in) {
    if (error != nullptr) {
      *error = "cannot open " + path;
    }
    return false;
  }
  out->clear();
  std::string line;
  while (std::getline(in, line)) {
    const std::string text = trim(line);
    if (!text.empty() && text[0] != '#') {
      out->push_back(text);
    }
  }
  return true;
}

}  // namespace ali
