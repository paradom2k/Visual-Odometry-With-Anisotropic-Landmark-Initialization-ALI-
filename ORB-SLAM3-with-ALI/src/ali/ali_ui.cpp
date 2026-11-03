// ali_ui -- the same run as `ali_run`, with a window.
//
// WHAT IT SHOWS, AND WHY THOSE FOUR THINGS
//   The acceptance criterion asks for a conf loader, a Run control, a view of the trajectory and a view of
//   the current frame with its features.  Those are the four panels below.  The fifth, the log strip, is
//   not asked for and is here because a run that does not initialise looks exactly like a run that is
//   thinking, and ORB-SLAM3 says which it is only on stdout.
//
// HOW IT IS PUT TOGETHER
//   Immediate-mode OpenGL through GLFW, in a 2D pixel canvas with the origin at the top left, and the 5x7
//   bitmap font the sibling repository already carries.  No shaders and no extension loader: the drawing
//   here is panels, polylines, one texture and glyph quads, all of which the fixed-function pipeline does,
//   and a UI that cannot be built because a GL function pointer did not resolve is a worse trade than one
//   that draws a rectangle the slow way.
//
//   The run happens on a worker thread, because the window has to keep responding while a 313-second drive
//   is being processed.  Everything the worker publishes -- the latest pose, the latest frame, the counters
//   -- goes through one mutex-guarded block that the drawing code copies under the lock and then uses
//   without it, so a slow frame cannot stall the estimator and the estimator cannot tear a frame.

#include "ali_conf.h"
#include "ali_files.h"
#include "ali_run.h"
#include "ali_sensors.h"
#include "common/geodesy.h"

#include "ORBextractor.h"


#include <memory>

#include <GLFW/glfw3.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

// ---------------------------------------------------------------- the font
// The 5x7 bitmap the sibling repository already carries, used as-is rather than regenerated: `kFontData`
// is 8 bytes per glyph for the printable ASCII range, and the layout (width, height, which bit is the
// left-most column) is named in that header so the two cannot drift apart.
// The bytes are inlined rather than included from the sibling repository's `font_data.h`: that header
// declares them `inline constexpr`, which is C++17, and this tree is built as C++14 -- the standard
// upstream ORB-SLAM3 is written to, which one binary cannot mix with another across a library boundary.
// 768 bytes of table is a smaller price than a second language standard.
static const unsigned char kFontBytes[768] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x00, 0x10, 0x00, 0x00,
    0x28, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x7c, 0x28, 0x28, 0x7c, 0x28, 0x00, 0x00,
    0x10, 0x3c, 0x50, 0x38, 0x14, 0x78, 0x10, 0x00, 0x64, 0x68, 0x10, 0x20, 0x2c, 0x4c, 0x00, 0x00,
    0x30, 0x48, 0x50, 0x20, 0x54, 0x48, 0x34, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0x10, 0x20, 0x20, 0x20, 0x10, 0x08, 0x00, 0x20, 0x10, 0x08, 0x08, 0x08, 0x10, 0x20, 0x00,
    0x00, 0x28, 0x10, 0x7c, 0x10, 0x28, 0x00, 0x00, 0x00, 0x10, 0x10, 0x7c, 0x10, 0x10, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x04, 0x08, 0x10, 0x20, 0x40, 0x00, 0x00, 0x00,
    0x38, 0x44, 0x4c, 0x54, 0x64, 0x44, 0x38, 0x00, 0x10, 0x30, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00,
    0x38, 0x44, 0x04, 0x18, 0x20, 0x40, 0x7c, 0x00, 0x7c, 0x08, 0x10, 0x18, 0x04, 0x44, 0x38, 0x00,
    0x08, 0x18, 0x28, 0x48, 0x7c, 0x08, 0x08, 0x00, 0x7c, 0x40, 0x78, 0x04, 0x04, 0x44, 0x38, 0x00,
    0x18, 0x20, 0x40, 0x78, 0x44, 0x44, 0x38, 0x00, 0x7c, 0x04, 0x08, 0x10, 0x20, 0x20, 0x20, 0x00,
    0x38, 0x44, 0x44, 0x38, 0x44, 0x44, 0x38, 0x00, 0x38, 0x44, 0x44, 0x3c, 0x04, 0x08, 0x30, 0x00,
    0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x20, 0x00,
    0x08, 0x10, 0x20, 0x40, 0x20, 0x10, 0x08, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x7c, 0x00, 0x00, 0x00,
    0x20, 0x10, 0x08, 0x04, 0x08, 0x10, 0x20, 0x00, 0x38, 0x44, 0x04, 0x18, 0x10, 0x00, 0x10, 0x00,
    0x38, 0x44, 0x5c, 0x54, 0x5c, 0x40, 0x38, 0x00, 0x10, 0x28, 0x44, 0x44, 0x7c, 0x44, 0x44, 0x00,
    0x78, 0x44, 0x44, 0x78, 0x44, 0x44, 0x78, 0x00, 0x38, 0x44, 0x40, 0x40, 0x40, 0x44, 0x38, 0x00,
    0x70, 0x48, 0x44, 0x44, 0x44, 0x48, 0x70, 0x00, 0x7c, 0x40, 0x40, 0x78, 0x40, 0x40, 0x7c, 0x00,
    0x7c, 0x40, 0x40, 0x78, 0x40, 0x40, 0x40, 0x00, 0x38, 0x44, 0x40, 0x5c, 0x44, 0x44, 0x3c, 0x00,
    0x44, 0x44, 0x44, 0x7c, 0x44, 0x44, 0x44, 0x00, 0x38, 0x10, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00,
    0x1c, 0x08, 0x08, 0x08, 0x08, 0x48, 0x30, 0x00, 0x44, 0x48, 0x50, 0x60, 0x50, 0x48, 0x44, 0x00,
    0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7c, 0x00, 0x44, 0x6c, 0x54, 0x54, 0x44, 0x44, 0x44, 0x00,
    0x44, 0x44, 0x64, 0x54, 0x4c, 0x44, 0x44, 0x00, 0x38, 0x44, 0x44, 0x44, 0x44, 0x44, 0x38, 0x00,
    0x78, 0x44, 0x44, 0x78, 0x40, 0x40, 0x40, 0x00, 0x38, 0x44, 0x44, 0x44, 0x54, 0x48, 0x34, 0x00,
    0x78, 0x44, 0x44, 0x78, 0x50, 0x48, 0x44, 0x00, 0x3c, 0x40, 0x40, 0x38, 0x04, 0x04, 0x78, 0x00,
    0x7c, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x38, 0x00,
    0x44, 0x44, 0x44, 0x44, 0x44, 0x28, 0x10, 0x00, 0x44, 0x44, 0x44, 0x54, 0x54, 0x6c, 0x44, 0x00,
    0x44, 0x44, 0x28, 0x10, 0x28, 0x44, 0x44, 0x00, 0x44, 0x44, 0x28, 0x10, 0x10, 0x10, 0x10, 0x00,
    0x7c, 0x04, 0x08, 0x10, 0x20, 0x40, 0x7c, 0x00, 0x38, 0x20, 0x20, 0x20, 0x20, 0x20, 0x38, 0x00,
    0x40, 0x20, 0x10, 0x08, 0x04, 0x00, 0x00, 0x00, 0x38, 0x08, 0x08, 0x08, 0x08, 0x08, 0x38, 0x00,
    0x10, 0x28, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00,
    0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x04, 0x3c, 0x44, 0x3c, 0x00,
    0x40, 0x40, 0x78, 0x44, 0x44, 0x44, 0x78, 0x00, 0x00, 0x00, 0x3c, 0x40, 0x40, 0x40, 0x3c, 0x00,
    0x04, 0x04, 0x3c, 0x44, 0x44, 0x44, 0x3c, 0x00, 0x00, 0x00, 0x38, 0x44, 0x7c, 0x40, 0x3c, 0x00,
    0x18, 0x24, 0x20, 0x78, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x3c, 0x44, 0x44, 0x3c, 0x04, 0x38,
    0x40, 0x40, 0x78, 0x44, 0x44, 0x44, 0x44, 0x00, 0x10, 0x00, 0x30, 0x10, 0x10, 0x10, 0x38, 0x00,
    0x08, 0x00, 0x18, 0x08, 0x08, 0x08, 0x48, 0x30, 0x40, 0x40, 0x48, 0x50, 0x60, 0x50, 0x48, 0x00,
    0x30, 0x10, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00, 0x00, 0x68, 0x54, 0x54, 0x54, 0x44, 0x00,
    0x00, 0x00, 0x78, 0x44, 0x44, 0x44, 0x44, 0x00, 0x00, 0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00,
    0x00, 0x00, 0x78, 0x44, 0x44, 0x78, 0x40, 0x40, 0x00, 0x00, 0x3c, 0x44, 0x44, 0x3c, 0x04, 0x04,
    0x00, 0x00, 0x58, 0x64, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x3c, 0x40, 0x38, 0x04, 0x78, 0x00,
    0x20, 0x20, 0x78, 0x20, 0x20, 0x24, 0x18, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0x4c, 0x34, 0x00,
    0x00, 0x00, 0x44, 0x44, 0x44, 0x28, 0x10, 0x00, 0x00, 0x00, 0x44, 0x54, 0x54, 0x54, 0x28, 0x00,
    0x00, 0x00, 0x44, 0x28, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0x3c, 0x04, 0x38,
    0x00, 0x00, 0x7c, 0x08, 0x10, 0x20, 0x7c, 0x00, 0x18, 0x20, 0x20, 0x60, 0x20, 0x20, 0x18, 0x00,
    0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x30, 0x08, 0x08, 0x0c, 0x08, 0x08, 0x30, 0x00,
    0x00, 0x00, 0x00, 0x34, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const std::uint8_t* const kGlyphs = kFontBytes;

constexpr int kGlyphW = 5;
constexpr int kGlyphH = 8;
constexpr int kGlyphAdvance = 5 + 1;   // glyph width plus one blank column
constexpr int kFirstChar = 32;
constexpr int kLastChar = 126;

// The glyph table is uploaded once as a small atlas: one textured quad per character.  The previous
// version expanded every lit pixel of every glyph into an independent quad; the configuration list and
// labels alone were ~10k immediate-mode quads per frame, which is what made the visible window's render
// (and the CPU it competed with the estimator for) grow with every UI refresh.
constexpr int kAtlasCols = 16;
constexpr int kAtlasRows = 6;
constexpr int kAtlasCellW = 8;  // 5 px glyph + transparent margin
constexpr int kAtlasCellH = 8;
constexpr int kAtlasW = kAtlasCols * kAtlasCellW;
constexpr int kAtlasH = kAtlasRows * kAtlasCellH;

GLuint g_font_texture = 0;

void ensureFontTexture() {
  if (g_font_texture != 0) {
    return;
  }
  std::vector<unsigned char> pixels(static_cast<std::size_t>(kAtlasW) * kAtlasH * 4, 0);
  for (int code = kFirstChar; code <= kLastChar; ++code) {
    const int idx = code - kFirstChar;
    const int cx = (idx % kAtlasCols) * kAtlasCellW;
    const int cy = (idx / kAtlasCols) * kAtlasCellH;
    const std::uint8_t* rows = &kGlyphs[idx * kGlyphH];
    for (int row = 0; row < kGlyphH; ++row) {
      for (int col = 0; col < kGlyphW; ++col) {
        if ((rows[row] >> (6 - col)) & 1) {
          const std::size_t o = (static_cast<std::size_t>(cy + row) * kAtlasW + (cx + col)) * 4;
          pixels[o + 0] = 255;
          pixels[o + 1] = 255;
          pixels[o + 2] = 255;
          pixels[o + 3] = 255;
        }
      }
    }
  }
  glGenTextures(1, &g_font_texture);
  glBindTexture(GL_TEXTURE_2D, g_font_texture);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, kAtlasW, kAtlasH, 0, GL_RGBA, GL_UNSIGNED_BYTE,
               pixels.data());
  glBindTexture(GL_TEXTURE_2D, 0);
}

void drawText(float x, float y, const std::string& text, float scale, float r, float g, float b) {
  if (text.empty()) {
    return;
  }
  ensureFontTexture();
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, g_font_texture);
  glColor4f(r, g, b, 1.0f);
  glBegin(GL_QUADS);
  float pen = x;
  for (char ch : text) {
    const int code = static_cast<unsigned char>(ch);
    if (code >= kFirstChar && code <= kLastChar) {
      const int idx = code - kFirstChar;
      const int cx = (idx % kAtlasCols) * kAtlasCellW;
      const int cy = (idx / kAtlasCols) * kAtlasCellH;
      const float u0 = static_cast<float>(cx) / kAtlasW;
      const float v0 = static_cast<float>(cy) / kAtlasH;
      const float u1 = static_cast<float>(cx + kGlyphW) / kAtlasW;
      const float v1 = static_cast<float>(cy + kGlyphH) / kAtlasH;
      const float px = pen;
      const float py = y;
      glTexCoord2f(u0, v0); glVertex2f(px, py);
      glTexCoord2f(u1, v0); glVertex2f(px + kGlyphW * scale, py);
      glTexCoord2f(u1, v1); glVertex2f(px + kGlyphW * scale, py + kGlyphH * scale);
      glTexCoord2f(u0, v1); glVertex2f(px, py + kGlyphH * scale);
    }
    pen += kGlyphAdvance * scale;
  }
  glEnd();
  glBindTexture(GL_TEXTURE_2D, 0);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_BLEND);
  glColor3f(1.0f, 1.0f, 1.0f);
}

float textWidth(const std::string& text, float scale) {
  return static_cast<float>(text.size()) * kGlyphAdvance * scale;
}

void fillRect(float x, float y, float w, float h, float r, float g, float b, float a = 1.0f) {
  glColor4f(r, g, b, a);
  glBegin(GL_QUADS);
  glVertex2f(x, y);
  glVertex2f(x + w, y);
  glVertex2f(x + w, y + h);
  glVertex2f(x, y + h);
  glEnd();
}

void strokeRect(float x, float y, float w, float h, float r, float g, float b) {
  glColor3f(r, g, b);
  glBegin(GL_LINE_LOOP);
  glVertex2f(x + 0.5f, y + 0.5f);
  glVertex2f(x + w - 0.5f, y + 0.5f);
  glVertex2f(x + w - 0.5f, y + h - 0.5f);
  glVertex2f(x + 0.5f, y + h - 0.5f);
  glEnd();
}

// ---------------------------------------------------------------- shared state
struct Shared {
  std::mutex mutex;
  /// Poses as x,y,z triples in ORB-SLAM3's WORLD frame, live and as reloaded from the written file.
  ///
  /// Triples and not pairs, and the reason is a bug this had: which two of the three axes hold the ground
  /// track depends on the MODE.  ORB-SLAM3's world frame is the first camera's frame for a visual run --
  /// x right, y down, z forward, so the ground track is in x-z -- but an inertial run re-aligns the whole
  /// map to gravity during initialisation, which puts z up and moves the ground track into x-y.  A view that
  /// picks the plane when it collects the poses has to know the mode in advance and cannot; one that keeps
  /// all three axes can decide from the poses themselves.  (Measured, full drive: fs_stereo spans
  /// 497.7 / 25.8 / 354.7 m on x / y / z, fs_stereo_inertial spans 489.8 / 351.2 / 4.3 m -- the small axis
  /// is a different one in each.)
  std::vector<float> trajectory;
  std::vector<float> final_xy;
  /// GNSS trajectories arrive in ECEF.  The UI displays them in a local ENU frame (east, north, up) so the
  /// ground plane is x-y, exactly like an RTKLIB plot; ECEF's y-z is not a horizontal plane.  The origin
  /// is the first collected pose and the stored triples are ENU after this transform.
  bool enu_valid = false;
  Eigen::Vector3d enu_origin = Eigen::Vector3d::Zero();
  Eigen::Matrix3d ecef_to_enu = Eigen::Matrix3d::Identity();
  std::vector<unsigned char> frame;///< the current left frame, 8-bit grayscale (fallback when no Mat is given)
  cv::Mat frame_mat;               ///< refcounted frame from the runner; no pixel copy between the two sides
  int frame_w = 0;
  int frame_h = 0;
  std::uint64_t frame_serial = 0;  ///< bumped when the frame changes; the texture upload is skipped otherwise
  std::vector<float> keypoints;    ///< x,y pairs for the displayed frame
  std::vector<unsigned char> frame_right;
  cv::Mat frame_right_mat;
  int frame_w_right = 0;
  int frame_h_right = 0;
  std::uint64_t frame_right_serial = 0;
  std::vector<float> keypoints_right;
  /// Bumped at the start of every run.  The image textures are static across runs; without this (and the
  /// frame clear) the right window kept the previous dataset's image after a run that has no right camera.
  std::uint64_t run_generation = 0;
  /// The dense LiDAR local cloud: x,y,z triples and one intensity per point, in the run's world frame,
  /// clipped around the vehicle by the runner and drawn under the trajectory.
  std::vector<float> cloud_xyz;
  std::vector<float> cloud_intensity;
  /// The whole map, one point per metre cube, accumulated from each rebuilt local cloud.  The trajectory
  /// view draws this under the path; the local cloud above changes as the vehicle moves on, and a picture
  /// of the route with only the last 60 m of map in it is not a picture of the route.
  ///
  /// The drawing reads an immutable SNAPSHOT, taken every few rebuilds: copying four hundred thousand
  /// points into every frame would hold this mutex for milliseconds sixty times a second and stall the
  /// run it is watching.
  std::shared_ptr<const std::vector<float> > map_xyz;
  std::shared_ptr<const std::vector<float> > map_intensity;
  std::vector<float> map_build_xyz;      ///< worker-only until a snapshot is taken
  std::vector<float> map_build_intensity;
  std::unordered_map<std::uint64_t, std::size_t> map_keys;
  std::size_t cloud_generation_seen = 0;
  std::size_t map_snapshots = 0;
  float pose_qw = 1.0f, pose_qx = 0.0f, pose_qy = 0.0f, pose_qz = 0.0f;
  std::size_t frame_index = 0;
  std::size_t frame_count = 0;
  double stamp = 0.0;
  std::atomic<bool> running{false};
  std::atomic<bool> finished{false};
  /// Raised by STOP and polled by `runConf` once per frame; cleared before every RUN.  Atomic because the
  /// button is read on the window thread and the run is on the worker.
  std::atomic<bool> stop_requested{false};
  std::string status;
  ali::RunResult result;
};

Shared g_shared;
std::thread g_worker;
std::unique_ptr<ORB_SLAM3::ORBextractor> g_extractor;
ali::Conf g_conf;
std::vector<std::string> g_confs;
int g_selected = 0;

/// Reset the GNSS display frame when a new trajectory is started or a file is reloaded.
void resetTrajectoryFrame() {
  g_shared.enu_valid = false;
}

/// Append one pose to a UI trajectory.  GNSS poses are ECEF; convert them to ENU once, with the first pose
/// as origin, so drawTrajectory's usual smallest-axis choice becomes x=east, y=north, z=up.
void appendTrajectoryPoint(std::vector<float>* out, double x, double y, double z) {
  if (!ali::isGnssMode(g_conf.mode)) {
    out->push_back(static_cast<float>(x));
    out->push_back(static_cast<float>(y));
    out->push_back(static_cast<float>(z));
    return;
  }
  const Eigen::Vector3d ecef(x, y, z);
  if (!ecef.allFinite()) {
    return;
  }
  if (!g_shared.enu_valid) {
    g_shared.enu_origin = ecef;
    const jo::Geodetic origin = jo::ecefToGeodetic(ecef);
    g_shared.ecef_to_enu = jo::ecefToEnuRotation(origin);
    g_shared.enu_valid = true;
  }
  const Eigen::Vector3d enu = g_shared.ecef_to_enu * (ecef - g_shared.enu_origin);
  out->push_back(static_cast<float>(enu.x()));
  out->push_back(static_cast<float>(enu.y()));
  out->push_back(static_cast<float>(enu.z()));
}
bool g_show_trajectory = true;
bool g_show_map = true;
bool g_show_left = true;
bool g_show_right = true;
bool g_mouse_was_down = false;
/// The right visualization column's width when the user has dragged the splitter, pixels.  Negative means
/// "follow the image aspect".  The divider sits between the trajectory and the image column and can be
/// dragged left or right; it is clamped to the trajectory's 380 px minimum and a 240 px column minimum.
float g_image_col_override = -1.0f;
bool g_splitter_dragging = false;
/// Configuration panel width when the user has dragged the left divider.  Negative means the default.
float g_panel_width_override = -1.0f;
bool g_panel_splitter_dragging = false;
std::string g_log_tail;

// ---------------------------------------------------------------- the worker
void progressCallback(const ali::ProgressInfo& info, void* user) {
  Shared* shared = static_cast<Shared*>(user);
  std::lock_guard<std::mutex> lock(shared->mutex);
  shared->frame_index = info.frame;
  shared->frame_count = info.frames;
  shared->stamp = info.stamp;
  if (info.tracked) {
    // All three axes; drawTrajectory picks the plane.  GNSS poses are ECEF and are converted to ENU here
    // so the displayed ground plane is east-north, not ECEF y-z.
    appendTrajectoryPoint(&shared->trajectory, info.x, info.y, info.z);
  }
  if (info.gray != nullptr && info.width > 0 && info.height > 0) {
    if (!info.gray_mat.empty()) {
      // Zero-copy: keep the Mat's header; the decoded buffer is refcounted and lives until the UI drops it.
      shared->frame_mat = info.gray_mat;
      shared->frame.clear();
    } else {
      const std::size_t bytes = static_cast<std::size_t>(info.width) * info.height;
      shared->frame.assign(info.gray, info.gray + bytes);
      shared->frame_mat.release();
    }
    shared->frame_w = info.width;
    shared->frame_h = info.height;
    ++shared->frame_serial;
    // The features are the TRACKER'S OWN now, handed over through the progress info.  This used to re-run a
    // full ORB extraction on every frame with a second extractor, which cost 44 % of the wall clock for the
    // same drive (78 s against 54 s over a 60 s window) and drew a set that the estimator had not matched.
    shared->keypoints.clear();
    if (info.keypoints_xy != nullptr && info.keypoint_count > 0) {
      shared->keypoints.assign(info.keypoints_xy, info.keypoints_xy + info.keypoint_count * 2);
    }
  }
  if (info.gray_right != nullptr && info.width_right > 0 && info.height_right > 0) {
    if (!info.gray_right_mat.empty()) {
      shared->frame_right_mat = info.gray_right_mat;
      shared->frame_right.clear();
    } else {
      const std::size_t bytes = static_cast<std::size_t>(info.width_right) * info.height_right;
      shared->frame_right.assign(info.gray_right, info.gray_right + bytes);
      shared->frame_right_mat.release();
    }
    shared->frame_w_right = info.width_right;
    shared->frame_h_right = info.height_right;
    ++shared->frame_right_serial;
    shared->keypoints_right.clear();
    if (info.keypoints_right_xy != nullptr && info.keypoint_right_count > 0) {
      shared->keypoints_right.assign(info.keypoints_right_xy,
                                     info.keypoints_right_xy + info.keypoint_right_count * 2);
    }
  }
  if (info.cloud_xyz != nullptr && info.cloud_count > 0) {
    // The GNSS runners produce ECEF cloud points, while the trajectory panel is local ENU.  Convert the
    // cloud with the same first-pose frame the trajectory used; otherwise the map is drawn at ECEF
    // coordinates, millions of metres away from the path, and looks absent.
    const float* cloud = info.cloud_xyz;
    std::vector<float> cloud_enu;
    if (ali::isGnssMode(g_conf.mode) && g_shared.enu_valid) {
      cloud_enu.resize(info.cloud_count * 3);
      for (std::size_t i = 0; i < info.cloud_count; ++i) {
        const Eigen::Vector3d ecef(info.cloud_xyz[i * 3], info.cloud_xyz[i * 3 + 1],
                                   info.cloud_xyz[i * 3 + 2]);
        const Eigen::Vector3d enu = g_shared.ecef_to_enu * (ecef - g_shared.enu_origin);
        cloud_enu[i * 3] = static_cast<float>(enu.x());
        cloud_enu[i * 3 + 1] = static_cast<float>(enu.y());
        cloud_enu[i * 3 + 2] = static_cast<float>(enu.z());
      }
      cloud = cloud_enu.data();
    }
    shared->cloud_xyz.assign(cloud, cloud + info.cloud_count * 3);
    shared->cloud_intensity.assign(info.cloud_intensity, info.cloud_intensity + info.cloud_count);
    if (info.cloud_generation != shared->cloud_generation_seen) {
      shared->cloud_generation_seen = info.cloud_generation;
      // Backup map policy: one point per cubic metre, capped.  The local cloud is a 60 m window that
      // moves with the vehicle; keeping its unseen points turns the panel into the map along the route.
      const std::size_t kMapPointCap = 4000000;
      const double kMapCell = 1.0;
      const std::size_t before = shared->map_build_intensity.size();
      for (std::size_t i = 0; i < info.cloud_count && shared->map_build_intensity.size() < kMapPointCap; ++i) {
        const float px = cloud[i * 3];
        const float py = cloud[i * 3 + 1];
        const float pz = cloud[i * 3 + 2];
        // A hash, not a 21-bit packing: the GNSS routes hand ECEF coordinates (|x| ~ 2.3e6 m),
        // which a 21-bit cell would wrap and silently merge.
        const long long ix = static_cast<long long>(std::floor(px / kMapCell));
        const long long iy = static_cast<long long>(std::floor(py / kMapCell));
        const long long iz = static_cast<long long>(std::floor(pz / kMapCell));
        const std::uint64_t key =
            (static_cast<std::uint64_t>(ix * 73856093LL) ^
             static_cast<std::uint64_t>(iy * 19349663LL) ^
             static_cast<std::uint64_t>(iz * 83492791LL));
        auto found = shared->map_keys.find(key);
        if (found == shared->map_keys.end()) {
          const std::size_t index = shared->map_build_intensity.size();
          shared->map_keys.emplace(key, index);
          shared->map_build_xyz.push_back(px);
          shared->map_build_xyz.push_back(py);
          shared->map_build_xyz.push_back(pz);
          shared->map_build_intensity.push_back(info.cloud_intensity[i]);
        } else if (info.cloud_intensity[i] > shared->map_build_intensity[found->second]) {
          // Keep the strongest return per 1 m cell.  The first point to land in a cell is often a weak
          // vegetation return; replacing it with the strongest one is what makes the white map show the
          // dark building/road edges instead of a uniform light speckle.
          shared->map_build_xyz[found->second * 3] = px;
          shared->map_build_xyz[found->second * 3 + 1] = py;
          shared->map_build_xyz[found->second * 3 + 2] = pz;
          shared->map_build_intensity[found->second] = info.cloud_intensity[i];
        }
      }
      // Publish every fifth rebuild, or at once when the map has just started: often enough that the
      // picture grows during a drive, rare enough that the copy is not what the run is spending time on.
      if (++shared->map_snapshots % 5 == 0 || before == 0) {
        shared->map_xyz = std::make_shared<const std::vector<float> >(shared->map_build_xyz);
        shared->map_intensity = std::make_shared<const std::vector<float> >(shared->map_build_intensity);
      }
    }
  }
}

void workerMain(ali::Conf conf, std::string repo_root) {
  g_shared.finished = false;
  {
    // Start every run from an empty image state.  The right camera exists only in some modes; without the
    // clear, a stereo run after an epipolar/mono one showed the previous dataset's right image under the
    // current frame header (the right fields were only ever overwritten, never emptied).  The generation
    // also makes the static GL textures upload the first frame of the new run even if its serial repeats.
    std::lock_guard<std::mutex> lock(g_shared.mutex);
    g_shared.frame.clear();
    g_shared.frame_mat.release();
    g_shared.frame_w = 0;
    g_shared.frame_h = 0;
    g_shared.frame_serial = 0;
    g_shared.keypoints.clear();
    g_shared.frame_right.clear();
    g_shared.frame_right_mat.release();
    g_shared.frame_w_right = 0;
    g_shared.frame_h_right = 0;
    g_shared.frame_right_serial = 0;
    g_shared.keypoints_right.clear();
    g_shared.frame_index = 0;
    g_shared.frame_count = 0;
    g_shared.stamp = 0.0;
    ++g_shared.run_generation;
  }
  const ali::RunResult result =
      ali::runConf(conf, repo_root, progressCallback, &g_shared, &g_shared.stop_requested);
  std::lock_guard<std::mutex> lock(g_shared.mutex);
  g_shared.result = result;
  g_shared.status = result.cancelled ? "stopped" : (result.ok ? "finished" : ("failed: " + result.error));
  {
    const double frames = static_cast<double>(std::max<std::size_t>(1, result.frames));
    std::printf("[I] ui timing: wall %.1f s (%.1f ms/frame), track %.1f s, io wait %.1f s (%.1f ms/frame), "
                "decode %.1f s%s, progress callback %.2f s (%.2f ms/frame)\n",
                result.wall_seconds, result.wall_seconds * 1000.0 / frames, result.seconds_track,
                result.seconds_io_wait, result.seconds_io_wait * 1000.0 / frames,
                result.seconds_decode, result.prefetched ? " (prefetched)" : "", result.seconds_progress,
                result.seconds_progress * 1000.0 / frames);
    std::fflush(stdout);
  }
  // The written trajectory is the authoritative one -- it carries the writer's own anchoring -- so the
  // finished picture is drawn from the file rather than from the live poses, which are in the raw world
  // frame.  Drawing the live ones afterwards would show a trajectory that is the right SHAPE and the wrong
  // place, and a reader comparing the window with the file would have no way to tell.
  g_shared.final_xy.clear();
  resetTrajectoryFrame();
  std::vector<std::string> lines;
  std::string error;
  if (ali::readLines(result.solution_path, &lines, &error)) {
    for (const std::string& line : lines) {
      double t = 0.0, x = 0.0, y = 0.0, z = 0.0;
      if (std::sscanf(line.c_str(), "%lf %lf %lf %lf", &t, &x, &y, &z) == 4) {
        appendTrajectoryPoint(&g_shared.final_xy, x, y, z);
      }
    }
  }
  g_shared.running = false;
  g_shared.finished = true;
}

// ---------------------------------------------------------------- conf discovery
std::string repoRoot() {
  std::string root = ali::repoRootFrom(ali::executablePath());
  return root;
}

void scanConfs(const std::string& root) {
  g_confs.clear();
  std::vector<std::string> files;
  std::string error;
  if (ali::listFiles(root + "/conf", &files, &error)) {
    for (const std::string& file : files) {
      if (file.size() > 5 && file.compare(file.size() - 5, 5, ".conf") == 0) {
        g_confs.push_back(file);
      }
    }
  }
}

// ---------------------------------------------------------------- the panels
/// The panel's geometry, computed in ONE place.
///
/// Cap the window's redraw rate.  The estimator runs on its own thread; the window only has to show the
/// latest state.  At 60 Hz the fixed text/image/trajectory drawing competed with the estimator's OpenMP
/// workers for CPU; 30 Hz halves that cost and is still smooth for a diagnostic view.
void paceUiFrame() {
  static double last = 0.0;
  constexpr double kTarget = 1.0 / 30.0;
  const double now = glfwGetTime();
  if (last > 0.0 && now - last < kTarget) {
    std::this_thread::sleep_for(std::chrono::duration<double>(kTarget - (now - last)));
  }
  last = glfwGetTime();
}

/// This exists because it was not, and the consequence was that RUN did nothing.  The button was drawn at a
/// y that depends on how many configuration files there are -- one row each -- while the click test asked
/// whether the cursor was between y = 300 and y = 340, a constant left over from an earlier version of the
/// panel.  With four configurations the button lands near y = 128, so every press fell outside the
/// rectangle being tested: the window drew, the button highlighted, and the click was discarded.  A
/// rectangle that is drawn and a rectangle that is tested have to be the same rectangle.
struct Layout {
  float panel = 300.0f;
  float list_top = 32.0f;   ///< y of the first configuration row, inside the panel
  // Panel width is adjustable by dragging the divider at its right edge.  The default is the old 300 px;
  // the clamp leaves room for the trajectory (and, in visual modes, the image column).
  static constexpr float kPanelMin = 220.0f;
  static constexpr float kPanelMax = 560.0f;
  float row = 22.0f;        ///< height of one configuration row
  float button_h = 30.0f;   ///< height of the RUN / STOP / RELOAD buttons
  float run_y = 0.0f;       ///< y of those buttons, from the row count
  float run_w = 0.0f;       ///< width of one of them
  float stop_x = 0.0f;      ///< left edge of STOP
  float reload_x = 0.0f;    ///< left edge of RELOAD
  float image_w = 560.0f;
  float view_y = 0.0f;      ///< y of the VIEW checkbox block (after the buttons and the info lines)

  void compute(std::size_t conf_count) {
    run_y = list_top + row * static_cast<float>(conf_count) + 8.0f;
    // Three across: two 6 px margins and two 6 px gaps.
    run_w = (panel - 24.0f) / 3.0f;
    stop_x = 6.0f + run_w + 6.0f;
    reload_x = stop_x + run_w + 6.0f;
    // buttons + 12 px gap + six 14 px info lines + 10 px, then the VIEW block.
    view_y = run_y + button_h + 12.0f + 6.0f * 14.0f + 10.0f;
  }
  /// The configuration row under `my`, or -1.
  int rowAt(double my) const {
    if (my < static_cast<double>(list_top) - 3.0) {
      return -1;
    }
    const int index = static_cast<int>((my - (static_cast<double>(list_top) - 3.0)) /
                                       static_cast<double>(row));
    return (index >= 0 && index < static_cast<int>(g_confs.size())) ? index : -1;
  }
  /// True when (mx, my) is inside the RUN button.
  bool onRun(double mx, double my) const {
    return mx >= 6.0 && mx < 6.0 + run_w && my >= run_y && my < run_y + button_h;
  }
  /// True when (mx, my) is inside STOP.  It only does anything while a run is in flight; the click handler
  /// and the drawing use the same rectangle, which is the failure this panel had once already.
  bool onStop(double mx, double my) const {
    return mx >= stop_x && mx < stop_x + run_w && my >= run_y && my < run_y + button_h;
  }
  /// ... and inside RELOAD, which is the rightmost third of the same row.
  bool onReload(double mx, double my) const {
    return mx >= reload_x && mx < reload_x + run_w && my >= run_y && my < run_y + button_h;
  }
};

/// Apply the user's panel width and recompute the row/button geometry.
void applyPanelWidth(Layout* layout, int window_width) {
  const float max_width = std::max(Layout::kPanelMin,
                                   std::min(Layout::kPanelMax, static_cast<float>(window_width) * 0.5f));
  if (g_panel_width_override > 0.0f) {
    layout->panel = std::min(std::max(g_panel_width_override, Layout::kPanelMin), max_width);
  } else {
    layout->panel = 300.0f;
  }
  layout->compute(g_confs.size());
}

/// Which of the three world axes is the vertical one, given their spans and the previous answer.
///
/// The axis a road vehicle barely uses is the vertical one, so this is the whole of the ground-plane
/// decision -- but it has to be a decision, because the convention is not the same for every mode.
/// ORB-SLAM3's world frame is the first camera's frame for a visual run (x right, y down, z forward, ground
/// track in x-z) and a gravity-aligned frame for an inertial one (z up, ground track in x-y).  A view that
/// commits to x-z draws a line for every inertial run, which is what this one did.
///
/// Pulled out of the drawing code so `--self-test` can exercise it: the failure is invisible until someone
/// runs a particular mode and looks at the window, and it has been got wrong once already.
///
/// `previous` is the last answer, or -1 if there is none.  A choice is only revised when the new smallest
/// span is decisively smaller than the next one, so the view does not flip while a run is starting and
/// every span is still noise.
int pickVerticalAxis(const float span[3], int previous) {
  int up = 0;
  if (span[1] < span[up]) up = 1;
  if (span[2] < span[up]) up = 2;
  if (previous < 0) return up;
  if (previous == up) return up;
  const float largest = std::max(span[0], std::max(span[1], span[2]));
  if (largest <= 5.0f) return previous;      // too little motion for the spans to mean anything yet
  float second = largest;
  for (int a = 0; a < 3; ++a) {
    if (a != up && span[a] < second) second = span[a];
  }
  return (second > 1.0f && span[up] < 0.35f * second) ? up : previous;
}

// Used to colour the map drawn under the trajectory.
void intensityColor(float v, float* r, float* g, float* b);
void beginClip(float x, float y, float w, float h);
void endClip();

void drawTrajectory(float x, float y, float w, float h, const Shared& view) {
  // Light map panel: the saved PCD is a grey map on white, and that is the readable picture.  The rest
  // of the UI stays dark; the map panel is deliberately a white sheet with dark reflectivity points.
  fillRect(x, y, w, h, 0.995f, 0.995f, 0.995f);
  strokeRect(x, y, w, h, 0.62f, 0.64f, 0.66f);

  const std::vector<float>& path = view.final_xy.empty() ? view.trajectory : view.final_xy;
  if (path.size() < 9) {
    drawText(x + 10, y + 8, "TRAJECTORY  ground plane, metres", 1.4f, 0.12f, 0.14f, 0.18f);
    drawText(x + 10, y + 30, "no pose yet", 1.6f, 0.32f, 0.34f, 0.38f);
    return;
  }

  // WHICH TWO OF THE THREE AXES ARE THE GROUND PLANE IS DECIDED HERE, FROM THE POSES.
  //
  // The axis a road vehicle barely uses is the vertical one, so the ground plane is the other two.  That
  // holds whatever convention the map is in, and the convention is not the same for every mode: ORB-SLAM3's
  // world frame is the first camera's frame for a visual run -- x right, y down, z forward -- while an
  // inertial run re-aligns the whole map to gravity during its initialisation, which puts z up and moves
  // the ground track from x-z into x-y.  Picking the plane when the poses are COLLECTED, as this used to,
  // means committing to x-z before the run has said which mode it is, and the inertial run then projects
  // onto its four-metre-tall thickness and draws a line.
  //
  // Measured, whole drive: fs_stereo spans 497.7 / 25.8 / 354.7 m on x / y / z; fs_stereo_inertial spans
  // 489.8 / 351.2 / 4.3 m.  The smallest span is y in the first and z in the second.
  float lo[3] = {path[0], path[1], path[2]};
  float hi[3] = {path[0], path[1], path[2]};
  for (std::size_t i = 0; i + 2 < path.size(); i += 3) {
    for (int a = 0; a < 3; ++a) {
      lo[a] = std::min(lo[a], path[i + a]);
      hi[a] = std::max(hi[a], path[i + a]);
    }
  }
  const float span[3] = {hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]};

  // Hysteresis, so the view does not flip while a run is starting and every span is still noise, and so a
  // choice is only ever revised when the new one is decisive.  -1 means "not chosen yet".
  static int chosen_up = -1;
  const bool gnss_enu = ali::isGnssMode(g_conf.mode);
  if (gnss_enu) {
    // GNSS poses were converted to local ENU when collected: x=east, y=north, z=up.  Fix the ground
    // plane to x-y; the generic smallest-span rule could otherwise choose another axis on a short run.
    chosen_up = 2;
  } else {
    chosen_up = pickVerticalAxis(span, chosen_up);
  }
  const int up = chosen_up;
  const int a0 = (up == 0) ? 1 : 0;
  const int a1 = (up == 2) ? 1 : 2;
  const char name0 = "xyz"[a0], name1 = "xyz"[a1], nameup = "xyz"[up];

  char header[160];
  if (gnss_enu) {
    std::snprintf(header, sizeof(header),
                  "TRAJECTORY  ground plane, ENU metres  (x=east, y=north)");
  } else {
    std::snprintf(header, sizeof(header),
                  "TRAJECTORY  ground plane, metres  (world %c-%c; %c is the small axis)", name0,
                  name1, nameup);
  }
  drawText(x + 10, y + 8, header, 1.4f, 0.12f, 0.14f, 0.18f);

  const float minx = lo[a0], maxx = hi[a0], miny = lo[a1], maxy = hi[a1];
  const float margin = 28.0f;
  const float spanx = std::max(1.0f, maxx - minx);
  const float spany = std::max(1.0f, maxy - miny);
  const float scale = std::min((w - 2 * margin) / spanx, (h - 2 * margin - 24) / spany);
  const float ox = x + w * 0.5f - 0.5f * (minx + maxx) * scale;
  // Screen y grows downward, so the north-up picture flips here.
  const float oy = y + 24 + (h - 24) * 0.5f + 0.5f * (miny + maxy) * scale;
  const auto map_x = [&](float v) { return ox + v * scale; };
  const auto map_y = [&](float v) { return oy - v * scale; };

  // The LiDAR map under the trajectory, in the same ground-plane transform: one top-down picture of the
  // route and the structures around it, coloured by intensity like the 3D panel.  The whole accumulated
  // map is drawn first, strided to a bounded vertex count, and the dense local cloud on top of it.
  const bool have_map = view.map_xyz && !view.map_xyz->empty() && view.map_intensity &&
                        view.map_intensity->size() == view.map_xyz->size() / 3;
  if (g_show_map && (have_map || !view.cloud_xyz.empty())) {
    beginClip(x, y + 20.0f, w, h - 20.0f);
    glPointSize(1.35f);
    glBegin(GL_POINTS);
    // Grey on white, like the PCD render: strong returns (buildings, metal) are dark; weak vegetation
    // returns stay light.  The old rainbow was a black-background palette and reads as noise here.
    if (have_map) {
      const std::vector<float>& map = *view.map_xyz;
      const std::vector<float>& tint = *view.map_intensity;
      const std::size_t stride = std::max<std::size_t>(1, tint.size() / 60000);
      for (std::size_t i = 0; i < tint.size(); i += stride) {
        float v = tint[i] / 70.0f;
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        const float tone = 0.58f - 0.50f * v;
        glColor3f(tone, tone, tone);
        glVertex2f(map_x(map[i * 3 + a0]), map_y(map[i * 3 + a1]));
      }
    }
    // When the accumulated map exists it already contains these points at one per cell.  Drawing the
    // 60 m local cloud on top again only doubled the density and turned the white map into noise; keep
    // the local cloud as the fallback for modes that have no accumulated map.
    if (!have_map) {
      glPointSize(1.1f);
      glColor3f(0.24f, 0.26f, 0.28f);
      for (std::size_t i = 0; i < view.cloud_intensity.size(); ++i) {
        float v = view.cloud_intensity[i] / 70.0f;
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        const float tone = 0.56f - 0.50f * v;
        glColor3f(tone, tone, tone);
        glVertex2f(map_x(view.cloud_xyz[i * 3 + a0]), map_y(view.cloud_xyz[i * 3 + a1]));
      }
    }
    glEnd();
    glPointSize(1.0f);
    endClip();
  }

  if (g_show_trajectory) {
  // A dark pass under the path: the map now covers the whole route, and a green line over green
  // vegetation is not a path a reader can follow.
  //
  // The path is decimated for drawing: this view redraws every frame, and drawing the whole history makes
  // the UI's own cost grow with the run (the estimator's per-frame time stayed flat while the wall time
  // rose to ~85 ms/frame at 5000 poses).  The auto-fit above still uses every pose; only the vertices are
  // bounded, at 1200 line segments.
  const std::size_t path_points = path.size() / 3;
  const std::size_t path_stride = std::max<std::size_t>(1, path_points / 1200);
  const auto path_vertex = [&](std::size_t k) {
    glVertex2f(map_x(path[k * 3 + a0]), map_y(path[k * 3 + a1]));
  };
  glColor3f(0.995f, 0.995f, 0.995f);
  glLineWidth(3.4f);
  glBegin(GL_LINE_STRIP);
  for (std::size_t k = 0; k < path_points; k += path_stride) {
    path_vertex(k);
  }
  if ((path_points - 1) % path_stride != 0) {
    path_vertex(path_points - 1);
  }
  glEnd();
  glColor3f(0.05f, 0.20f, 0.52f);
  glLineWidth(1.4f);
  glBegin(GL_LINE_STRIP);
  for (std::size_t k = 0; k < path_points; k += path_stride) {
    path_vertex(k);
  }
  if ((path_points - 1) % path_stride != 0) {
    path_vertex(path_points - 1);
  }
  glEnd();
  glLineWidth(1.0f);
  // The start, marked, because a trajectory with no origin is hard to read against anything.
  glColor3f(0.85f, 0.25f, 0.05f);
  glPointSize(7.0f);
  glBegin(GL_POINTS);
  glVertex2f(map_x(path[a0]), map_y(path[a1]));
  glEnd();
  glPointSize(1.0f);
  }  // g_show_trajectory
  char buffer[192];
  std::snprintf(buffer, sizeof(buffer),
                "span %.1f m %c, %.1f m %c   %zu pose(s)   %.2f px/m",
                maxx - minx, name0, maxy - miny, name1, path.size() / 3, scale);
  drawText(x + 10, y + h - 18, buffer, 1.4f, 0.30f, 0.32f, 0.36f);

  // A scale bar.  The trajectory is auto-fitted, so it ALWAYS fills the panel and the picture alone cannot
  // say whether the drive was ten metres or two kilometres -- which is how a collapsed trajectory came to
  // look like a plausible one.  The bar is in metres and labelled, so the size comes from the figure and
  // not only from the line of text underneath it.
  {
    const double target_px = 120.0;
    double metres = target_px / scale;
    const double decade = std::pow(10.0, std::floor(std::log10(metres)));
    metres = std::max(1.0, std::floor(metres / decade) * decade);
    const float bar_px = static_cast<float>(metres * scale);
    const float bx = x + w - 24.0f - bar_px;
    const float by = y + h - 40.0f;
    glColor3f(0.22f, 0.24f, 0.28f);
    glBegin(GL_LINES);
    glVertex2f(bx, by); glVertex2f(bx + bar_px, by);
    glVertex2f(bx, by - 5.0f); glVertex2f(bx, by + 5.0f);
    glVertex2f(bx + bar_px, by - 5.0f); glVertex2f(bx + bar_px, by + 5.0f);
    glEnd();
    std::snprintf(buffer, sizeof(buffer), "%.0f m", metres);
    drawText(bx + bar_px * 0.5f - textWidth(buffer, 1.4f) * 0.5f, by - 20.0f, buffer, 1.4f, 0.22f,
             0.24f, 0.28f);
  }
}

void drawFrame(float x, float y, float w, float h, const Shared& view, bool right) {
  fillRect(x, y, w, h, 0.09f, 0.10f, 0.13f);
  strokeRect(x, y, w, h, 0.26f, 0.29f, 0.34f);
  const cv::Mat* mat = right ? &view.frame_right_mat : &view.frame_mat;
  const std::vector<unsigned char>* fallback = right ? &view.frame_right : &view.frame;
  const int stored_w = right ? view.frame_w_right : view.frame_w;
  const int stored_h = right ? view.frame_h_right : view.frame_h;
  const std::uint64_t serial = right ? view.frame_right_serial : view.frame_serial;
  const std::vector<float>& kps = right ? view.keypoints_right : view.keypoints;
  char buffer[192];
  std::snprintf(buffer, sizeof(buffer), "CURRENT FRAME (%s)   %zu / %zu   t %.3f   %zu feature(s)",
                right ? "RIGHT" : "LEFT", view.frame_index + 1, view.frame_count, view.stamp,
                kps.size() / 2);
  drawText(x + 10, y + 8, buffer, 1.5f, 0.75f, 0.80f, 0.86f);
  const bool use_mat = !mat->empty();
  if (!use_mat && fallback->empty()) {
    drawText(x + 10, y + 30, right ? "no right frame yet" : "no left frame yet", 1.6f, 0.55f, 0.58f,
             0.62f);
    return;
  }
  const int frame_w = use_mat ? mat->cols : stored_w;
  const int frame_h = use_mat ? mat->rows : stored_h;
  const unsigned char* frame_pixels = use_mat ? mat->data : fallback->data();
  const float top = y + 28.0f;
  const float avail_w = w - 20.0f;
  const float avail_h = h - 40.0f;
  const float scale = std::min(avail_w / static_cast<float>(frame_w),
                               avail_h / static_cast<float>(frame_h));
  const float iw = frame_w * scale;
  const float ih = frame_h * scale;
  const float ix = x + (w - iw) * 0.5f;
  const float iy = top;

  // The image as a texture.  Nearest filtering and no mipmaps: this is a diagnostic view and a blurred
  // frame tells the reader less about the features than an aliased one.  Each side owns one texture.
  struct CameraTexture {
    GLuint texture = 0;
    std::uint64_t serial = ~0ull;
    std::uint64_t generation = ~0ull;
    int width = 0;
    int height = 0;
  };
  static CameraTexture left_texture;
  static CameraTexture right_texture;
  CameraTexture& tex = right ? right_texture : left_texture;
  if (tex.texture == 0) {
    glGenTextures(1, &tex.texture);
  }
  glBindTexture(GL_TEXTURE_2D, tex.texture);
  if (serial != tex.serial || view.run_generation != tex.generation || frame_w != tex.width ||
      frame_h != tex.height) {
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, frame_w, frame_h, 0, GL_LUMINANCE,
                 GL_UNSIGNED_BYTE, frame_pixels);
    tex.serial = serial;
    tex.generation = view.run_generation;
    tex.width = frame_w;
    tex.height = frame_h;
  }
  glEnable(GL_TEXTURE_2D);
  glColor3f(1.0f, 1.0f, 1.0f);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0f, 0.0f); glVertex2f(ix, iy);
  glTexCoord2f(1.0f, 0.0f); glVertex2f(ix + iw, iy);
  glTexCoord2f(1.0f, 1.0f); glVertex2f(ix + iw, iy + ih);
  glTexCoord2f(0.0f, 1.0f); glVertex2f(ix, iy + ih);
  glEnd();
  glDisable(GL_TEXTURE_2D);

  glColor3f(0.20f, 1.0f, 0.45f);
  glLineWidth(1.0f);
  constexpr float kCross = 2.5f;
  // The live right camera now really draws its ~1000 ORB keypoints every frame (before the right-camera
  // feed fix the texture was stale, so this overlay was effectively 61 points).  Immediate-mode crosses for
  // both cameras at 30 Hz became a measurable UI cost; draw a representative subset and leave the header's
  // feature count untouched.  The trajectory/estimator are unaffected.
  constexpr std::size_t kMaxOverlay = 300;
  const std::size_t kp_count = kps.size() / 2;
  const std::size_t stride = kp_count > kMaxOverlay ? (kp_count + kMaxOverlay - 1) / kMaxOverlay : 1;
  glBegin(GL_LINES);
  for (std::size_t i = 0; i + 1 < kps.size(); i += 2 * stride) {
    const float px = ix + kps[i] * scale;
    const float py = iy + kps[i + 1] * scale;
    glVertex2f(px - kCross, py - kCross);
    glVertex2f(px + kCross, py + kCross);
    glVertex2f(px - kCross, py + kCross);
    glVertex2f(px + kCross, py - kCross);
  }
  glEnd();
}

// ---------------------------------------------------------------- the LiDAR map
int g_window_height = 0;

/// Clip the following drawing to a panel.  The scissor box is in GL's bottom-left origin, this UI's
/// coordinates are top-left, hence the flip.  Without this the 3D projection, which spans the whole
/// viewport, drew the cloud over the trajectory and the configuration panel.
void beginClip(float x, float y, float w, float h) {
  glEnable(GL_SCISSOR_TEST);
  glScissor(static_cast<GLint>(x), static_cast<GLint>(static_cast<float>(g_window_height) - (y + h)),
            static_cast<GLsizei>(std::max(0.0f, w)), static_cast<GLsizei>(std::max(0.0f, h)));
}

void endClip() { glDisable(GL_SCISSOR_TEST); }

/// Reflectivity -> colour.  Deliberately NOT a height ramp: a LiDAR map that reads like a scene is
/// coloured by INTENSITY, because the surface's reflectivity separates what height cannot -- vegetation
/// absorbs the beam (low, green), asphalt sits in the middle (orange), buildings and metal return a lot
/// (white), and retroreflective signs clip the top of the ramp (magenta).  Six stops, linear between.
void intensityColor(float v, float* r, float* g, float* b) {
  struct Stop {
    float t, r, g, b;
  };
  static const Stop kStops[] = {
      {0.00f, 0.04f, 0.22f, 0.05f},  // dark green: the darkest returns
      {0.18f, 0.12f, 0.65f, 0.10f},  // vegetation
      {0.34f, 0.70f, 0.62f, 0.06f},  // grass / low returns
      {0.55f, 1.00f, 0.42f, 0.05f},  // asphalt / road surface
      {0.80f, 0.94f, 0.90f, 0.86f},  // buildings, metal
      {1.00f, 1.00f, 0.06f, 0.85f},  // retroreflective signs, plates
  };
  const int n = static_cast<int>(sizeof(kStops) / sizeof(kStops[0]));
  if (v <= kStops[0].t) {
    *r = kStops[0].r; *g = kStops[0].g; *b = kStops[0].b;
    return;
  }
  for (int i = 1; i < n; ++i) {
    if (v <= kStops[i].t) {
      const float u = (v - kStops[i - 1].t) / (kStops[i].t - kStops[i - 1].t);
      *r = kStops[i - 1].r + u * (kStops[i].r - kStops[i - 1].r);
      *g = kStops[i - 1].g + u * (kStops[i].g - kStops[i - 1].g);
      *b = kStops[i - 1].b + u * (kStops[i].b - kStops[i - 1].b);
      return;
    }
  }
  *r = kStops[n - 1].r; *g = kStops[n - 1].g; *b = kStops[n - 1].b;
}

void drawCheckbox(float x, float y, const char* label, bool checked) {
  fillRect(x, y, 12.0f, 12.0f, 0.16f, 0.18f, 0.22f);
  strokeRect(x, y, 12.0f, 12.0f, 0.45f, 0.50f, 0.58f);
  if (checked) {
    fillRect(x + 3.0f, y + 3.0f, 6.0f, 6.0f, 0.30f, 0.90f, 0.55f);
  }
  drawText(x + 20.0f, y, label, 1.4f, checked ? 0.85f : 0.55f, checked ? 0.90f : 0.58f,
           checked ? 0.94f : 0.62f);
}

int viewCheckboxAt(const Layout& layout, double mx, double my) {
  if (mx < 6.0 || mx > static_cast<double>(layout.panel) - 6.0) {
    return -1;
  }
  for (int i = 0; i < 4; ++i) {
    const double row_y = static_cast<double>(layout.view_y) + 18.0 + i * 18.0;
    if (my >= row_y - 2.0 && my < row_y + 16.0) {
      return i;
    }
  }
  return -1;
}

bool handleViewCheckbox(const Layout& layout, double mx, double my) {
  const int i = viewCheckboxAt(layout, mx, my);
  if (i < 0) {
    return false;
  }
  bool* flags[4] = {&g_show_trajectory, &g_show_map, &g_show_left, &g_show_right};
  *flags[i] = !*flags[i];
  return true;
}

void drawPanelStatic(float x, float y, float w, float h, const Layout& layout) {
  fillRect(x, y, w, h, 0.12f, 0.13f, 0.17f);
  strokeRect(x, y, w, h, 0.26f, 0.29f, 0.34f);
  drawText(x + 12, y + 10, "CONFIGURATION", 1.6f, 0.75f, 0.80f, 0.86f);

  float cursor = y + layout.list_top;
  for (std::size_t i = 0; i < g_confs.size(); ++i) {
    const bool selected = (static_cast<int>(i) == g_selected);
    if (selected) {
      fillRect(x + 6, cursor - 3, w - 12, 20, 0.20f, 0.34f, 0.52f);
    }
    std::string name = g_confs[i];
    const std::size_t slash = name.find_last_of("/\\");
    if (slash != std::string::npos) {
      name = name.substr(slash + 1);
    }
    drawText(x + 14, cursor + 2, name, 1.6f, selected ? 1.0f : 0.72f, selected ? 1.0f : 0.76f,
             selected ? 1.0f : 0.82f);
    cursor += layout.row;
  }
}

void drawPanelLive(float x, float y, float w, float h, const Layout& layout, const Shared& view) {
  float cursor = y + layout.run_y;
  const bool busy = view.running.load();
  fillRect(x + 6, cursor, layout.run_w, layout.button_h,
           busy ? 0.22f : 0.16f, busy ? 0.26f : 0.48f, busy ? 0.28f : 0.30f);
  drawText(x + 18, cursor + 10, "RUN", 2.0f, busy ? 0.5f : 0.95f, busy ? 0.5f : 0.98f,
           busy ? 0.5f : 0.95f);
  fillRect(x + layout.stop_x, cursor, layout.run_w, layout.button_h,
           busy ? 0.52f : 0.20f, busy ? 0.20f : 0.22f, busy ? 0.22f : 0.26f);
  drawText(x + layout.stop_x + 12, cursor + 10, "STOP", 1.8f, busy ? 1.0f : 0.45f,
           busy ? 0.72f : 0.48f, busy ? 0.68f : 0.52f);
  fillRect(x + layout.reload_x, cursor, layout.run_w, layout.button_h, 0.22f, 0.24f, 0.28f);
  drawText(x + layout.reload_x + 12, cursor + 10, "RELOAD", 1.6f, 0.85f, 0.88f, 0.92f);
  cursor += layout.button_h + 12.0f;

  char buffer[256];
  std::snprintf(buffer, sizeof(buffer), "mode      %s", ali::modeName(g_conf.mode));
  drawText(x + 12, cursor, buffer, 1.5f, 0.72f, 0.76f, 0.82f);
  cursor += 14.0f;
  const bool gnss_run = ali::isGnssMode(g_conf.mode);
  const bool lidar_run = ali::isLidarMode(g_conf.mode);
  std::snprintf(buffer, sizeof(buffer), "input     %s",
                gnss_run ? "none" : (lidar_run ? "LiDAR" : "images"));
  drawText(x + 12, cursor, buffer, 1.5f, 0.72f, 0.76f, 0.82f);
  cursor += 14.0f;
  const bool inertial =
      ali::isInertialMode(g_conf.mode) && g_conf.imu_source != ali::ImuSource::kNone;
  std::snprintf(buffer, sizeof(buffer), "inertial  %s",
                inertial ? (g_conf.imu_source == ali::ImuSource::kBag ? "bag" : "file") : "none");
  drawText(x + 12, cursor, buffer, 1.5f, 0.72f, 0.76f, 0.82f);
  cursor += 14.0f;
  const std::size_t shown_frame =
      std::min(view.frame_count, view.frame_index + (busy ? 1u : 0u));
  std::snprintf(buffer, sizeof(buffer), "frames    %zu / %zu", shown_frame, view.frame_count);
  drawText(x + 12, cursor, buffer, 1.5f, 0.72f, 0.76f, 0.82f);
  cursor += 14.0f;
  const std::size_t shown_poses =
      std::max(view.final_xy.size() / 3, view.trajectory.size() / 3);
  std::snprintf(buffer, sizeof(buffer), "poses     %zu", shown_poses);
  drawText(x + 12, cursor, buffer, 1.5f, 0.72f, 0.76f, 0.82f);
  cursor += 14.0f;
  std::snprintf(buffer, sizeof(buffer), "solution  sol/%s.txt", "");
  drawText(x + 12, cursor, buffer, 1.4f, 0.55f, 0.58f, 0.62f);

  const float vy = y + layout.view_y;
  drawText(x + 12, vy, "VIEW", 1.4f, 0.55f, 0.58f, 0.62f);
  drawCheckbox(x + 12, vy + 18.0f, "TRAJECTORY", g_show_trajectory);
  drawCheckbox(x + 12, vy + 36.0f, "LIDAR MAP", g_show_map);
  if (!gnss_run) {
    drawCheckbox(x + 12, vy + 54.0f, "LEFT CAMERA", g_show_left);
    drawCheckbox(x + 12, vy + 72.0f, "RIGHT CAMERA", g_show_right);
  }
}

// The configuration panel is static apart from the buttons and the per-frame counters.  Render the
// background, title and list once into a texture (via the back buffer; no FBO loader needed), and blit
// that single quad every frame; the live layer is drawn on top.  Invalidated by selection, list size,
// window height or panel width.
GLuint g_panel_texture = 0;
int g_panel_tex_w = 0;
int g_panel_tex_h = 0;
bool g_panel_cache_valid = false;
int g_panel_cache_selected = -1;
std::size_t g_panel_cache_confs = 0;
int g_panel_cache_height = 0;
int g_panel_cache_width = 0;

void copyPanelTexture(int w, int h) {
  if (w <= 0 || h <= 0) {
    return;
  }
  if (g_panel_texture == 0) {
    glGenTextures(1, &g_panel_texture);
  }
  glBindTexture(GL_TEXTURE_2D, g_panel_texture);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  if (w != g_panel_tex_w || h != g_panel_tex_h) {
    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 0, 0, w, h, 0);
    g_panel_tex_w = w;
    g_panel_tex_h = h;
  } else {
    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, w, h);
  }
  glBindTexture(GL_TEXTURE_2D, 0);
}

void drawPanelCached(float w, float h) {
  if (g_panel_texture == 0 || !g_panel_cache_valid) {
    return;
  }
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, g_panel_texture);
  glColor3f(1.0f, 1.0f, 1.0f);
  // The copy read the back buffer's bottom-left origin; flip v so the panel is upright in this
  // top-left-origin ortho.
  glBegin(GL_QUADS);
  glTexCoord2f(0.0f, 1.0f); glVertex2f(0.0f, 0.0f);
  glTexCoord2f(1.0f, 1.0f); glVertex2f(w, 0.0f);
  glTexCoord2f(1.0f, 0.0f); glVertex2f(w, h);
  glTexCoord2f(0.0f, 0.0f); glVertex2f(0.0f, h);
  glEnd();
  glBindTexture(GL_TEXTURE_2D, 0);
  glDisable(GL_TEXTURE_2D);
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
  // The LLVM OpenMP runtime reads these before the first parallel region.  Passive workers do not spin
  // between the estimator's many short parallel loops.
  _putenv_s("OMP_WAIT_POLICY", "PASSIVE");
  _putenv_s("KMP_BLOCKTIME", "0");
#endif
  const std::string root = ali::repoRootFrom(ali::executablePath());
  scanConfs(root);
  if (g_confs.empty()) {
    std::fprintf(stderr, "[E] no .conf files under %s/conf\n", root.c_str());
    return 1;
  }
  {
    std::string error;
    if (!ali::loadConf(g_confs[0], &g_conf, &error)) {
      std::fprintf(stderr, "[E] %s: %s\n", g_confs[0].c_str(), error.c_str());
      return 1;
    }
  }

  // `--snapshot <conf> [--seconds N] [--out file.ppm]`: draw the window for N seconds with the given
  // configuration running, then read the framebuffer.  The window is hidden and no input is needed, so
  // the drawing can be checked by a test instead of by a person looking at it -- which is how a cloud
  // spilling out of its panel, and a trajectory view without a map under it, both passed a build.
  std::string snapshot_conf;
  std::string snapshot_out;
  double snapshot_seconds = 0.0;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--snapshot") == 0 && i + 1 < argc) {
      snapshot_conf = argv[++i];
    } else if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
      snapshot_seconds = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
      snapshot_out = argv[++i];
    }
  }
  const bool snapshot = !snapshot_conf.empty();
  if (snapshot) {
    std::string error;
    if (!ali::loadConf(snapshot_conf, &g_conf, &error)) {
      std::fprintf(stderr, "[E] %s: %s\n", snapshot_conf.c_str(), error.c_str());
      return 1;
    }
    if (snapshot_out.empty()) {
      snapshot_out = "snapshot.ppm";
    }
    if (snapshot_seconds <= 0.0) {
      snapshot_seconds = 20.0;
    }
    // The snapshot is a fixed-duration run, not just a wall-clock stop: the LiDAR display pre-pass and
    // the estimator both use the same end stamp, so a 120 s snapshot does not map the whole sequence.
    g_conf.duration = snapshot_seconds;
    const auto base = [](const std::string& path) {
      const std::size_t slash = path.find_last_of("/\\");
      return slash == std::string::npos ? path : path.substr(slash + 1);
    };
    const std::string wanted = base(snapshot_conf);
    for (std::size_t i = 0; i < g_confs.size(); ++i) {
      if (base(g_confs[i]) == wanted) {
        g_selected = static_cast<int>(i);
      }
    }
  }

  // `--self-test`: every control is clicked AT ITS OWN CENTRE, through the same hit test the window uses,
  // and the result printed.  It exists because a control whose rectangle and whose hit test disagree cannot
  // be found by reading either of them: the drawing was right, the action was right, and the press was
  // discarded between the two.  That is what RUN did -- the test asked for y between 300 and 340 while the
  // button was drawn near y = 128, a constant left over from an earlier panel -- and it survived a build, a
  // launch and a screenshot of a correct-looking window.  A click that lands is now a measurement.
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--self-test") != 0) {
      continue;
    }
    Layout layout;
    // The self-test does not open a window; 1440 is the window width the snapshot/self-test uses.
    applyPanelWidth(&layout, 1440);
    std::printf("self-test: %zu configuration(s), panel %.0f px wide\n", g_confs.size(), layout.panel);
    std::printf("  list rows      y %.1f .. %.1f, row height %.1f\n", layout.list_top,
                layout.list_top + layout.row * static_cast<float>(g_confs.size()), layout.row);
    std::printf("  RUN button     x %.1f .. %.1f   y %.1f .. %.1f\n", 6.0, 6.0 + layout.run_w,
                layout.run_y, layout.run_y + layout.button_h);
    std::printf("  STOP button    x %.1f .. %.1f   y %.1f .. %.1f\n", layout.stop_x,
                layout.stop_x + layout.run_w, layout.run_y, layout.run_y + layout.button_h);
    std::printf("  RELOAD button  x %.1f .. %.1f   y %.1f .. %.1f\n", layout.reload_x,
                layout.reload_x + layout.run_w, layout.run_y, layout.run_y + layout.button_h);
    int failures = 0;
    for (std::size_t i = 0; i < g_confs.size(); ++i) {
      const double centre_y = layout.list_top + layout.row * static_cast<double>(i) + layout.row * 0.5;
      const int hit = layout.rowAt(centre_y);
      const bool ok = hit == static_cast<int>(i);
      if (!ok) {
        ++failures;
      }
      std::printf("  %s row %zu at its centre y %.1f selects row %d\n", ok ? "PASS" : "FAIL", i, centre_y,
                  hit);
    }
    {
      const double cx = 6.0 + layout.run_w * 0.5;
      const double cy = layout.run_y + layout.button_h * 0.5;
      const bool ok = layout.onRun(cx, cy) && !layout.onReload(cx, cy) && layout.rowAt(cy) < 0;
      if (!ok) {
        ++failures;
      }
      std::printf("  %s RUN centre (%.1f, %.1f) hits RUN and nothing else\n", ok ? "PASS" : "FAIL", cx,
                  cy);
    }
    {
      const double cx = layout.stop_x + layout.run_w * 0.5;
      const double cy = layout.run_y + layout.button_h * 0.5;
      const bool ok = layout.onStop(cx, cy) && !layout.onRun(cx, cy) && !layout.onReload(cx, cy) &&
                      layout.rowAt(cy) < 0;
      if (!ok) {
        ++failures;
      }
      std::printf("  %s STOP centre (%.1f, %.1f) hits STOP and nothing else\n", ok ? "PASS" : "FAIL", cx,
                  cy);
    }
    {
      const double cx = layout.reload_x + layout.run_w * 0.5;
      const double cy = layout.run_y + layout.button_h * 0.5;
      const bool ok = layout.onReload(cx, cy) && !layout.onRun(cx, cy) && !layout.onStop(cx, cy) &&
                      layout.rowAt(cy) < 0;
      if (!ok) {
        ++failures;
      }
      std::printf("  %s RELOAD centre (%.1f, %.1f) hits RELOAD and nothing else\n", ok ? "PASS" : "FAIL",
                  cx, cy);
    }
    // ... and a point that is meant to hit nothing, so "everything hits something" cannot pass by accident.
    {
      const bool ok = !layout.onRun(layout.panel - 2.0, layout.run_y + 1.0);
      if (!ok) {
        ++failures;
      }
      std::printf("  %s the panel's right edge is not a button\n", ok ? "PASS" : "FAIL");
    }
    // The ground-plane choice, on the two span triples this project actually measured over a full drive.
    // This is the one drawing decision that depends on the MODE, it is invisible until a run is looked at,
    // and it has been wrong once -- so it gets a test rather than a comment.
    {
      const float visual[3] = {497.73f, 25.80f, 354.69f};      // fs_stereo:      y is the small axis
      const float inertial[3] = {489.80f, 351.23f, 4.26f};     // fs_stereo_inertial: z is
      const int up_visual = pickVerticalAxis(visual, -1);
      const int up_inertial = pickVerticalAxis(inertial, -1);
      const bool ok = (up_visual == 1) && (up_inertial == 2);
      if (!ok) {
        ++failures;
      }
      std::printf("  %s ground plane: visual map picks %c (want y), inertial map picks %c (want z)\n",
                  ok ? "PASS" : "FAIL", "xyz"[up_visual], "xyz"[up_inertial]);
    }
    // ... and that a noisy early choice is not revised by noise, only by a decisive span.
    {
      const float noise[3] = {3.0f, 3.1f, 2.9f};
      const bool ok = pickVerticalAxis(noise, 1) == 1;
      if (!ok) {
        ++failures;
      }
      std::printf("  %s ground plane: an undecided span keeps the previous choice\n", ok ? "PASS" : "FAIL");
    }
    std::printf("self-test: %d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
  }

  // `--run-test <conf> [--frames N]`: run a configuration through the WORKER THREAD and the progress
  // callback, with no window at all.
  //
  // It exists because the progress callback is the one piece of this program the headless runner never
  // exercises -- `ali_run` passes a null callback, so every measurement taken through it, including all
  // four full-drive runs, ran without the code that copies the frame and detects its features.  A crash
  // there is therefore invisible to every test this project had, and it is the code a click on RUN
  // reaches first.  This path runs exactly that, in a terminal, where the failure can be read.
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--run-test") != 0) {
      continue;
    }
    std::string conf_path;
    double seconds = 3.0;
    double stop_after = 0.0;
    for (int k = i + 1; k < argc; ++k) {
      if (std::strcmp(argv[k], "--duration") == 0 && k + 1 < argc) {
        seconds = std::atof(argv[++k]);
      } else if (std::strcmp(argv[k], "--stop-after") == 0 && k + 1 < argc) {
        stop_after = std::atof(argv[++k]);
      } else if (conf_path.empty()) {
        conf_path = argv[k];
      }
    }
    if (conf_path.empty()) {
      std::fprintf(stderr, "[E] --run-test needs a configuration file\n");
      return 2;
    }
    ali::Conf conf;
    std::string error;
    if (!ali::loadConf(conf_path, &conf, &error)) {
      std::fprintf(stderr, "[E] %s: %s\n", conf_path.c_str(), error.c_str());
      return 2;
    }
    conf.duration = seconds;
    std::printf("run-test: %s (%s) for %.1f s\n", conf_path.c_str(), ali::modeName(conf.mode), seconds);
    std::printf("run-test: building the ORB extractor (%d features, %.2f scale, %d levels)\n",
                conf.features, static_cast<double>(conf.scale_factor), conf.levels);
    std::fflush(stdout);
    g_extractor.reset(new ORB_SLAM3::ORBextractor(conf.features, conf.scale_factor, conf.levels,
                                                 conf.ini_th_fast, conf.min_th_fast));
    std::printf("run-test: extracting features from one frame before anything else runs\n");
    std::fflush(stdout);
    {
      // The extractor, on a frame of this configuration's own first image.  Done FIRST and on its own, so
      // that a failure here is reported as a failure of the extractor rather than as a crash somewhere
      // inside the estimator.
      std::vector<ali::FrameRef> frames;
      std::string frame_error;
      if (ali::collectFrames(conf, &frames, &frame_error) && !frames.empty()) {
        cv::Mat image;
        if (ali::readFrame(conf, frames.front(), &image, &frame_error)) {
          std::vector<cv::KeyPoint> keypoints;
          cv::Mat descriptors;
          // Two elements, always.  `ORBextractor::operator()` indexes `vLappingArea[0]` and [1]
      // unconditionally (ORBextractor.cc:1291) to decide which keypoints fall in the band the
      // stereo pair shares, so an EMPTY vector is an out-of-bounds read and an access violation.
      // ORB-SLAM3's own caller passes `{x0, x1}` (Frame.cc:436); `{0, 0}` is the same call with a
      // band of zero width, which is what a monocular frame and this feature view both want.
      std::vector<int> lapping = {0, 0};
          (*g_extractor)(image, cv::Mat(), keypoints, descriptors, lapping);
          std::printf("run-test: %zu keypoint(s) from %dx%d\n", keypoints.size(), image.cols,
                      image.rows);
        } else {
          std::printf("run-test: cannot read the first frame: %s\n", frame_error.c_str());
        }
      } else {
        std::printf("run-test: cannot list frames: %s\n", frame_error.c_str());
      }
      std::fflush(stdout);
    }
    std::printf("run-test: running through the worker and the progress callback\n");
    std::fflush(stdout);
    g_shared.running = true;
    g_shared.stop_requested = false;
    // `--stop-after`: raises the same flag the STOP button raises, from a second thread, so the
    // cancellation path is exercised headlessly -- otherwise the only way it is ever tested is a person
    // clicking, which is how the RUN button came to do nothing for a build and a screenshot.
    std::thread stopper;
    if (stop_after > 0.0) {
      std::printf("run-test: STOP will be raised after %.1f s\n", stop_after);
      std::fflush(stdout);
      stopper = std::thread([stop_after]() {
        std::this_thread::sleep_for(std::chrono::duration<double>(stop_after));
        g_shared.stop_requested = true;
      });
    }
    workerMain(conf, root);
    if (stopper.joinable()) {
      stopper.join();
    }
    std::printf("run-test: finished, %zu pose(s), %zu frame(s) shown%s\n",
                g_shared.result.poses, g_shared.frame_index + 1,
                g_shared.result.cancelled ? " (stopped on request)" : "");
    std::printf("run-test: %s\n", g_shared.status.c_str());
    return g_shared.result.ok ? 0 : 1;
  }

  if (glfwInit() != GLFW_TRUE) {
    std::fprintf(stderr, "[E] glfwInit failed\n");
    return 1;
  }
  if (snapshot) {
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  }
  GLFWwindow* window = glfwCreateWindow(1440, 900, "ALI_GVIL -- visual, inertial and LiDAR odometry", nullptr, nullptr);
  if (window == nullptr) {
    std::fprintf(stderr, "[E] cannot create the window\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  std::printf("[I] GL_VERSION %s\n", reinterpret_cast<const char*>(glGetString(GL_VERSION)));

  double snapshot_start = 0.0;
  bool snapshot_run_started = false;
  bool snapshot_written = false;
  if (snapshot) {
    {
      std::lock_guard<std::mutex> lock(g_shared.mutex);
      g_shared.trajectory.clear();
      g_shared.final_xy.clear();
      resetTrajectoryFrame();
      g_shared.frame.clear();
      g_shared.keypoints.clear();
      g_shared.cloud_xyz.clear();
      g_shared.cloud_intensity.clear();
      g_shared.map_xyz.reset();
      g_shared.map_intensity.reset();
      g_shared.map_build_xyz.clear();
      g_shared.map_build_intensity.clear();
      g_shared.map_keys.clear();
      g_shared.cloud_generation_seen = 0;
      g_shared.map_snapshots = 0;
      g_shared.frame_index = 0;
      g_shared.frame_count = 0;
      g_shared.status = "running";
    }
    g_extractor.reset(new ORB_SLAM3::ORBextractor(g_conf.features, g_conf.scale_factor, g_conf.levels,
                                                  g_conf.ini_th_fast, g_conf.min_th_fast));
    g_shared.stop_requested = false;
    g_shared.running = true;
    g_worker = std::thread(workerMain, g_conf, root);
    snapshot_start = glfwGetTime();
    std::printf("[I] snapshot: running %s for %.1f s\n", g_conf.path.c_str(), snapshot_seconds);
    std::fflush(stdout);
  }

  while (glfwWindowShouldClose(window) == 0) {
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, width, height, 0.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glClearColor(0.07f, 0.08f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    g_window_height = height;

    Layout layout;
    applyPanelWidth(&layout, width);
    // A copy of everything the worker publishes, taken once under the lock.  Drawing outside the lock is
    // what keeps a stalled frame from stalling the estimator.
    Shared view;
    {
      std::lock_guard<std::mutex> lock(g_shared.mutex);
      view.trajectory = g_shared.trajectory;
      view.final_xy = g_shared.final_xy;
      view.frame = g_shared.frame;
      view.frame_mat = g_shared.frame_mat;
      view.frame_w = g_shared.frame_w;
      view.frame_h = g_shared.frame_h;
      view.frame_serial = g_shared.frame_serial;
      view.keypoints = g_shared.keypoints;
      view.frame_right = g_shared.frame_right;
      view.frame_right_mat = g_shared.frame_right_mat;
      view.frame_w_right = g_shared.frame_w_right;
      view.frame_h_right = g_shared.frame_h_right;
      view.frame_right_serial = g_shared.frame_right_serial;
      view.keypoints_right = g_shared.keypoints_right;
      view.run_generation = g_shared.run_generation;
      view.cloud_xyz = g_shared.cloud_xyz;
      view.cloud_intensity = g_shared.cloud_intensity;
      view.map_xyz = g_shared.map_xyz;
      view.map_intensity = g_shared.map_intensity;
      view.frame_index = g_shared.frame_index;
      view.frame_count = g_shared.frame_count;
      view.stamp = g_shared.stamp;
      view.status = g_shared.status;
    }
    view.running = g_shared.running.load();

    // A LiDAR run has no image: the trajectory takes the whole width to the right of the configuration
    // panel and the map is drawn under the path.  A visual run keeps the two image panels on the right,
    // their width following the images rather than a constant: each panel gets half the window height,
    // so the width that fills that half is (panel height - chrome) * aspect.  The trajectory keeps at
    // least 380 px and the column never shrinks below 240 px.
    const bool lidar_run = (g_conf.mode == ali::Mode::kLidar || g_conf.mode == ali::Mode::kLidarInertial);
    // The GNSS-only / GNSS+IMU routes have no image stream; their trajectory fills the view.  LVI
    // publishes its latest stereo pair through ProgressInfo, so it keeps the two camera columns.
    const bool gnss_visual = g_conf.mode == ali::Mode::kGnssInertialEpipolar ||
                             g_conf.mode == ali::Mode::kGnssInertialLidarEpipolar;
    const bool no_image_run = lidar_run || (ali::isGnssMode(g_conf.mode) && !gnss_visual);
    const int cam_count = no_image_run ? 0 : (g_show_left ? 1 : 0) + (g_show_right ? 1 : 0);
    float image_col_w = 0.0f;
    float max_col_w = 0.0f;
    float map_left = 0.0f;
    const auto computeImageColumn = [&]() {
      image_col_w = 0.0f;
      max_col_w = 0.0f;
      if (cam_count > 0) {
        const float camera_h = static_cast<float>(height) / static_cast<float>(cam_count);
        const float aspect = (view.frame_w > 0 && view.frame_h > 0)
                                 ? static_cast<float>(view.frame_w) / static_cast<float>(view.frame_h)
                                 : 4.0f / 3.0f;
        image_col_w = (camera_h - 40.0f) * aspect + 20.0f;
        max_col_w = std::max(240.0f, static_cast<float>(width) - layout.panel - 380.0f);
        if (g_image_col_override > 0.0f) {
          image_col_w = g_image_col_override;
        }
        image_col_w = std::min(std::max(image_col_w, 240.0f), max_col_w);
      }
      map_left = static_cast<float>(width) - image_col_w;
    };
    computeImageColumn();

    // The mouse.  A PRESS is an edge, not a level: reading the button's state every frame means holding
    // it down repeats the click sixty times a second, which for a row selection is harmless and for RUN
    // starts a second run the moment the first finishes.
    double mx = 0.0;
    double my = 0.0;
    glfwGetCursorPos(window, &mx, &my);
    const bool mouse_down = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool clicked = mouse_down && !g_mouse_was_down;
    g_mouse_was_down = mouse_down;

    // Left divider: the configuration panel width.  Handle it before the image splitter so the same
    // frame's trajectory/image layout uses the new width.
    bool hovering_panel_splitter = false;
    bool panel_splitter_consumed = false;
    {
      const double distance = std::fabs(mx - static_cast<double>(layout.panel));
      hovering_panel_splitter = mx >= 0.0 && distance <= 6.0;
      if (clicked && hovering_panel_splitter) {
        g_panel_splitter_dragging = true;
        panel_splitter_consumed = true;
      }
      if (g_panel_splitter_dragging && mouse_down) {
        const float max_panel = std::max(
            Layout::kPanelMin,
            std::min(Layout::kPanelMax,
                     (cam_count > 0 ? static_cast<float>(width) - 620.0f
                                    : static_cast<float>(width) - 380.0f)));
        g_panel_width_override =
            std::min(std::max(static_cast<float>(mx), Layout::kPanelMin), max_panel);
        applyPanelWidth(&layout, width);
        computeImageColumn();
        hovering_panel_splitter = true;
        panel_splitter_consumed = true;
      }
      if (!mouse_down) {
        g_panel_splitter_dragging = false;
      }
    }

    // The splitter between the trajectory and the image column.  A press within 6 px of the divider
    // starts a drag; the column width follows the cursor until the button is released.
    bool hovering_splitter = false;
    if (cam_count > 0 && !panel_splitter_consumed && mx >= static_cast<double>(layout.panel)) {
      const double distance = std::fabs(mx - static_cast<double>(map_left));
      hovering_splitter = distance <= 6.0;
      if (clicked && hovering_splitter) {
        g_splitter_dragging = true;
      }
      if (g_splitter_dragging && mouse_down) {
        g_image_col_override =
            std::min(std::max(static_cast<float>(width) - static_cast<float>(mx), 240.0f), max_col_w);
        image_col_w = g_image_col_override;
        map_left = static_cast<float>(width) - image_col_w;
        hovering_splitter = true;
      }
      if (!mouse_down) {
        g_splitter_dragging = false;
      }
    }

    const bool panel_cache_dirty = !g_panel_cache_valid ||
        height != g_panel_cache_height || g_selected != g_panel_cache_selected ||
        g_confs.size() != g_panel_cache_confs ||
        static_cast<int>(layout.panel) != g_panel_cache_width;
    if (panel_cache_dirty) {
      drawPanelStatic(0.0f, 0.0f, layout.panel, static_cast<float>(height), layout);
      copyPanelTexture(static_cast<int>(layout.panel), height);
      g_panel_cache_valid = true;
      g_panel_cache_selected = g_selected;
      g_panel_cache_confs = g_confs.size();
      g_panel_cache_height = height;
      g_panel_cache_width = static_cast<int>(layout.panel);
    }
    drawPanelCached(layout.panel, static_cast<float>(height));
    drawPanelLive(0.0f, 0.0f, layout.panel, static_cast<float>(height), layout, view);
    drawTrajectory(layout.panel, 0, static_cast<float>(width) - layout.panel - image_col_w,
                   static_cast<float>(height), view);
    // The left divider handle.
    {
      const float brightness = (g_panel_splitter_dragging || hovering_panel_splitter) ? 0.95f : 0.42f;
      fillRect(layout.panel - 1.5f, 0.0f, 3.0f, static_cast<float>(height), brightness,
               brightness * 0.85f, brightness * 0.55f);
    }
    if (cam_count > 0) {
      const float camera_h = static_cast<float>(height) / static_cast<float>(cam_count);
      if (cam_count == 2) {
        drawFrame(map_left, 0.0f, image_col_w, camera_h, view, false);
        drawFrame(map_left, camera_h, image_col_w, static_cast<float>(height) - camera_h, view, true);
      } else if (g_show_left) {
        drawFrame(map_left, 0.0f, image_col_w, static_cast<float>(height), view, false);
      } else {
        drawFrame(map_left, 0.0f, image_col_w, static_cast<float>(height), view, true);
      }
      // The splitter: a 3 px handle, brighter while it is hovered or dragged.
      const float brightness = (g_splitter_dragging || hovering_splitter) ? 0.95f : 0.42f;
      fillRect(map_left - 1.5f, 0.0f, 3.0f, static_cast<float>(height), brightness, brightness * 0.85f,
               brightness * 0.55f);
    }
    (void)layout.image_w;

    if (clicked && !panel_splitter_consumed && mx >= 0.0 && mx < layout.panel) {
      {
        const int index = layout.rowAt(my);
        if (index >= 0 && index != g_selected) {
          g_selected = index;
          std::string error;
          if (!ali::loadConf(g_confs[g_selected], &g_conf, &error)) {
            std::fprintf(stderr, "[E] %s: %s\n", g_confs[g_selected].c_str(), error.c_str());
          } else {
            std::printf("[I] loaded %s (%s)\n", g_conf.path.c_str(), ali::modeName(g_conf.mode));
          }
        } else if (layout.onRun(mx, my)) {
          {
            if (g_worker.joinable()) {
              g_worker.join();
            }
            // A fresh state for a fresh run, and an extractor built from THIS configuration's feature
            // settings so the feature view and the estimator are looking for the same thing.
            {
              std::lock_guard<std::mutex> lock(g_shared.mutex);
              g_shared.trajectory.clear();
              g_shared.final_xy.clear();
              resetTrajectoryFrame();
              g_shared.frame.clear();
              g_shared.keypoints.clear();
              g_shared.cloud_xyz.clear();
              g_shared.cloud_intensity.clear();
              g_shared.map_xyz.reset();
              g_shared.map_intensity.reset();
              g_shared.map_build_xyz.clear();
              g_shared.map_build_intensity.clear();
              g_shared.map_keys.clear();
              g_shared.cloud_generation_seen = 0;
              g_shared.map_snapshots = 0;
              g_shared.frame_index = 0;
              g_shared.frame_count = 0;
              g_shared.status = "running";
            }
            g_extractor.reset(new ORB_SLAM3::ORBextractor(g_conf.features, g_conf.scale_factor, g_conf.levels,
                                               g_conf.ini_th_fast, g_conf.min_th_fast));
            // Cleared here rather than after the previous run, so a STOP that was pressed while the worker
            // was already finishing cannot leak into the next run and cancel it on its first frame.
            g_shared.stop_requested = false;
            g_shared.running = true;
            g_worker = std::thread(workerMain, g_conf, root);
            std::printf("[I] running %s (%s)\n", g_conf.path.c_str(), ali::modeName(g_conf.mode));
            std::fflush(stdout);
          }
        } else if (layout.onStop(mx, my)) {
          // Non-blocking: the flag is polled at the next frame boundary, the worker shuts the estimator down
          // and writes what it has, and the window keeps drawing until it is done.  Joining here would
          // freeze the window for the length of ORB-SLAM3's shutdown.
          if (g_shared.running.load() && !g_shared.stop_requested.load()) {
            g_shared.stop_requested = true;
            std::lock_guard<std::mutex> lock(g_shared.mutex);
            g_shared.status = "stopping";
            std::printf("[I] stop requested\n");
            std::fflush(stdout);
          }
        } else if (handleViewCheckbox(layout, mx, my)) {
          // The visualization flags changed; the next frame draws the new layout.
        } else if (layout.onReload(mx, my) && !view.running.load()) {
          // RELOAD re-reads the selected configuration and the trajectory it already has on disk, without
          // running anything -- which is what makes a finished run's picture come back after the window has
          // been closed and reopened, and what lets the four delivered configurations be compared by
          // clicking between them.
          std::string error;
          if (!ali::loadConf(g_confs[g_selected], &g_conf, &error)) {
            std::fprintf(stderr, "[E] %s: %s\n", g_confs[g_selected].c_str(), error.c_str());
          } else {
            std::lock_guard<std::mutex> lock(g_shared.mutex);
            g_shared.trajectory.clear();
            g_shared.final_xy.clear();
            resetTrajectoryFrame();
            std::vector<std::string> lines;
            std::string read_error;
            if (ali::readLines(g_conf.resolvedSolution(root), &lines, &read_error)) {
              for (const std::string& line : lines) {
                double t = 0.0, x = 0.0, y = 0.0, z = 0.0;
                if (std::sscanf(line.c_str(), "%lf %lf %lf %lf", &t, &x, &y, &z) == 4) {
                  appendTrajectoryPoint(&g_shared.final_xy, x, y, z);
                }
              }
            }
            g_shared.status = g_shared.final_xy.empty()
                                  ? std::string("no trajectory on disk for this configuration")
                                  : ("loaded " + std::to_string(g_shared.final_xy.size() / 3) + " pose(s)");
            std::printf("[I] reloaded %s: %zu pose(s)\n", g_conf.resolvedSolution(root).c_str(),
                        g_shared.final_xy.size() / 3);
            std::fflush(stdout);
          }
        }
        // A click that is not a run is a no-op rather than an error: the panel has no other control.
        paceUiFrame();
        glfwPollEvents();
        glfwSwapBuffers(window);
        continue;
      }
    }

    if (view.status == "running") {
      drawText(layout.panel + 10, height - 36.0f, "running...", 1.5f, 0.90f, 0.85f, 0.40f);
    } else if (!view.status.empty()) {
      drawText(layout.panel + 10, height - 36.0f, view.status, 1.5f, 0.70f, 0.85f, 0.70f);
    }
    if (snapshot && !snapshot_run_started) {
      std::lock_guard<std::mutex> lock(g_shared.mutex);
      if (!g_shared.trajectory.empty()) {
        snapshot_run_started = true;
        snapshot_start = glfwGetTime();
      }
    }
    if (snapshot && snapshot_run_started && !snapshot_written &&
        glfwGetTime() - snapshot_start >= snapshot_seconds) {
      // Read the frame that was just drawn, before the swap.  GL's origin is bottom-left, so the rows
      // are written top-first; PPM keeps this to no image dependency at all.
      glFinish();
      std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3);
      glPixelStorei(GL_PACK_ALIGNMENT, 1);
      glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
      if (FILE* file = std::fopen(snapshot_out.c_str(), "wb")) {
        std::fprintf(file, "P6\n%d %d\n255\n", width, height);
        for (int row = height - 1; row >= 0; --row) {
          std::fwrite(&pixels[static_cast<std::size_t>(row) * static_cast<std::size_t>(width) * 3], 1,
                      static_cast<std::size_t>(width) * 3, file);
        }
        std::fclose(file);
      }
      std::printf("[I] snapshot: wrote %s (%dx%d), %zu cloud point(s), %zu pose(s)\n", snapshot_out.c_str(),
                  width, height, view.cloud_xyz.size() / 3,
                  view.final_xy.size() / 3 + view.trajectory.size() / 3);
      std::fflush(stdout);
      snapshot_written = true;
      g_shared.stop_requested = true;  // do not make join() wait for the whole sequence
      glfwSetWindowShouldClose(window, 1);
    }
    paceUiFrame();
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  if (g_worker.joinable()) {
    g_worker.join();
  }
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
