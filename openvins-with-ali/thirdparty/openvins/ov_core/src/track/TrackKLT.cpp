/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "TrackKLT.h"

#include "Grider_FAST.h"
#include "Grider_GRID.h"
#include "cam/CamBase.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "utils/opencv_lambda_body.h"
#include "utils/print.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <unordered_set>
#include <limits>

using namespace ov_core;

// ===================================================================================================================
// Experiment C: rank the detected candidates by a statistic instead of the detector's own response.
//
// The integration point is deliberately the smallest one: Grider_GRID still decides WHICH cells get
// features (the coverage constraint the plan requires), and the only thing that changes is which
// candidates fill a cell.  The pool is over-sampled 3x so that the cell's choice is not a FAST
// preselection, and the same over-sampling is used for every branch including the random one, so the
// branches differ in the ranking and in nothing else.
//
// The orientation of each statistic is the one the offline table used (fixed from fs_full in Phase A):
// lambda2 and Foerstner prefer large values, CSR and the characteristic scale are inverted (the offline
// AUC of both is below 0.5, i.e. the direction the previous paper published points the wrong way on
// this data).
// ===================================================================================================================
namespace {

inline void eig2(double a, double b, double c, double &l1, double &l2) {
  const double tr = 0.5 * (a + c);
  const double d = std::sqrt(std::max(0.0, 0.25 * (a - c) * (a - c) + b * b));
  l1 = tr + d;
  l2 = tr - d;
}

/// Eigenvalues of the structure tensor over a (2*half+1)^2 window centred on (x,y).
bool tensor_eig(const cv::Mat &Ix, const cv::Mat &Iy, int x, int y, int half, double &l1, double &l2) {
  if (x - half < 0 || y - half < 0 || x + half >= Ix.cols || y + half >= Ix.rows)
    return false;
  double a = 0.0, b = 0.0, c = 0.0;
  for (int dy = -half; dy <= half; ++dy) {
    const float *px = Ix.ptr<float>(y + dy) + (x - half);
    const float *py = Iy.ptr<float>(y + dy) + (x - half);
    for (int dx = 0; dx < 2 * half + 1; ++dx) {
      const double gx = px[dx], gy = py[dx];
      a += gx * gx;
      b += gx * gy;
      c += gy * gy;
    }
  }
  eig2(a, b, c, l1, l2);
  return true;
}

/// Anisotropic pixel covariance C ~= ref * M^{-1}, normalised to a running reference
/// and clipped so the SLAM initialization cannot be dominated by one direction.
bool tensor_cov(const cv::Mat &Ix, const cv::Mat &Iy, int x, int y, int half, double ref,
                Eigen::Matrix2d &C) {
  if (x - half < 0 || y - half < 0 || x + half >= Ix.cols || y + half >= Ix.rows)
    return false;
  double a = 0.0, b = 0.0, c = 0.0;
  for (int dy = -half; dy <= half; ++dy) {
    const float *px = Ix.ptr<float>(y + dy) + (x - half);
    const float *py = Iy.ptr<float>(y + dy) + (x - half);
    for (int dx = 0; dx < 2 * half + 1; ++dx) {
      const double gx = px[dx], gy = py[dx];
      a += gx * gx;
      b += gx * gy;
      c += gy * gy;
    }
  }
  double l1 = 0.0, l2 = 0.0;
  eig2(a, b, c, l1, l2);
  if (l1 <= 1e-12 || l2 <= 1e-12 || ref <= 1e-12)
    return false;
  const double theta = 0.5 * std::atan2(2.0 * b, a - c);
  const double ct = std::cos(theta), st = std::sin(theta);
  const double e1 = std::min(4.0, std::max(0.25, ref / l1));
  const double e2 = std::min(4.0, std::max(0.25, ref / l2));
  const Eigen::Matrix2d U((Eigen::Matrix2d() << ct, -st, st, ct).finished());
  C = U * Eigen::Vector2d(e1, e2).asDiagonal() * U.transpose();
  return true;
}

/// Per-frame gradients the statistic needs.  Level 0 is the tracker's own equalised image, i.e. exactly
/// the pixels the detector saw.
struct CandStats {
  cv::Mat Ix, Iy;                  // level 0
  cv::Mat hIx, hIy;                // level 1 (half resolution), for CSR
  std::vector<cv::Mat> bIx, bIy;   // one pair per sigma, for the characteristic-scale branch
};

const std::vector<double> &cscale_sigmas() {
  static const std::vector<double> s = {0.5, 0.7071, 1.0, 1.4142, 2.0, 2.8284, 4.0, 5.6569, 8.0, 11.3137};
  return s;
}

void prepare_stats(CandStats &st, const std::vector<cv::Mat> &pyr, bool with_scales) {
  const cv::Mat &img = pyr.at(0);
  cv::Sobel(img, st.Ix, CV_32F, 1, 0, 3);
  cv::Sobel(img, st.Iy, CV_32F, 0, 1, 3);
  cv::Sobel(pyr.at(1), st.hIx, CV_32F, 1, 0, 3);
  cv::Sobel(pyr.at(1), st.hIy, CV_32F, 0, 1, 3);
  st.bIx.clear();
  st.bIy.clear();
  if (!with_scales)
    return;
  cv::Mat blur, pad;
  for (double s : cscale_sigmas()) {
    cv::GaussianBlur(img, blur, cv::Size(0, 0), s, s);
    cv::Mat gx, gy;
    cv::Sobel(blur, gx, CV_32F, 1, 0, 3);
    cv::Sobel(blur, gy, CV_32F, 0, 1, 3);
    st.bIx.push_back(gx);
    st.bIy.push_back(gy);
  }
}

/// The ranking score: larger is better.  Bounds checked, so a candidate too close to the border scores
/// -inf and loses its cell to one that can be scored.
double cand_score(TrackKLT::SelectMode mode, const CandStats &st, int x, int y, uint32_t salt) {
  double l1 = 0.0, l2 = 0.0;
  switch (mode) {
  case TrackKLT::SelectMode::L2:
    if (!tensor_eig(st.Ix, st.Iy, x, y, 7, l1, l2))
      return -std::numeric_limits<double>::infinity();
    return l2;
  case TrackKLT::SelectMode::FOSTNER:
    if (!tensor_eig(st.Ix, st.Iy, x, y, 7, l1, l2))
      return -std::numeric_limits<double>::infinity();
    return (l1 + l2 > 1e-12) ? (l1 * l2) / (l1 + l2) : 0.0;
  case TrackKLT::SelectMode::CSR: {
    if (!tensor_eig(st.Ix, st.Iy, x, y, 7, l1, l2))
      return -std::numeric_limits<double>::infinity();
    const double full = l2;
    double d1 = 0.0, d2 = 0.0;
    // half resolution: the same physical footprint is a 7x7 window at (x/2, y/2)
    if (!tensor_eig(st.hIx, st.hIy, x / 2, y / 2, 3, d1, d2))
      return -std::numeric_limits<double>::infinity();
    const double csr = d2 / std::max(full, 1e-12);
    return -csr;   // offline orientation: smaller CSR survives better
  }
  case TrackKLT::SelectMode::CSCALE: {
    // gamma=1 characteristic scale: argmax_sigma sigma * l2(sigma); offline orientation is inverted
    int best = 0;
    double bestv = -1.0;
    for (size_t k = 0; k < st.bIx.size(); ++k) {
      if (!tensor_eig(st.bIx[k], st.bIy[k], x, y, 7, l1, l2))
        return -std::numeric_limits<double>::infinity();
      const double v = cscale_sigmas()[k] * l2;
      if (v > bestv) {
        bestv = v;
        best = (int)k;
      }
    }
    return -cscale_sigmas()[best];
  }
  case TrackKLT::SelectMode::RANDOM:
    // a deterministic pseudo-random score: the null branch must be reproducible
    salt = salt * 1664525u + 1013904223u;
    return (double)(salt >> 8) / 16777216.0;
  default:
    return 0.0;
  }
}

} // namespace

int TrackKLT::select_start() {
  static const int s = []() {
    const char *v = std::getenv("OV_SELECT_START");
    return v ? std::max(0, std::atoi(v)) : 0;
  }();
  return s;
}

TrackKLT::SelectMode TrackKLT::active_select_mode() const {
  return frame_count_ >= (size_t)select_start() ? select_mode() : SelectMode::FAST;
}

TrackKLT::SelectMode TrackKLT::select_mode() {
  static const SelectMode m = []() {
    const char *s = std::getenv("OV_SELECT");
    const std::string v = s ? std::string(s) : std::string();
    if (v == "fastpool")
      return SelectMode::FASTPOOL;
    if (v == "random")
      return SelectMode::RANDOM;
    if (v == "l2")
      return SelectMode::L2;
    if (v == "fostner")
      return SelectMode::FOSTNER;
    if (v == "csr")
      return SelectMode::CSR;
    if (v == "cscale")
      return SelectMode::CSCALE;
    if (v == "cp1")
      return SelectMode::C_P1;
    if (v == "cp3")
      return SelectMode::C_P3;
    if (v == "parallax")
      return SelectMode::PARALLAX;
    if (v == "replace")
      return SelectMode::REPLACE;
    return SelectMode::FAST;
  }();
  return m;
}

void TrackKLT::feed_new_camera(const CameraData &message) {

  // Error check that we have all the data
  if (message.sensor_ids.empty() || message.sensor_ids.size() != message.images.size() || message.images.size() != message.masks.size()) {
    PRINT_ERROR(RED "[ERROR]: MESSAGE DATA SIZES DO NOT MATCH OR EMPTY!!!\n" RESET);
    PRINT_ERROR(RED "[ERROR]:   - message.sensor_ids.size() = %zu\n" RESET, message.sensor_ids.size());
    PRINT_ERROR(RED "[ERROR]:   - message.images.size() = %zu\n" RESET, message.images.size());
    PRINT_ERROR(RED "[ERROR]:   - message.masks.size() = %zu\n" RESET, message.masks.size());
    std::exit(EXIT_FAILURE);
  }

  // Preprocessing steps that we do not parallelize
  // NOTE: DO NOT PARALLELIZE THESE!
  // NOTE: These seem to be much slower if you parallelize them...
  rT1 = boost::posix_time::microsec_clock::local_time();
  size_t num_images = message.images.size();
  for (size_t msg_id = 0; msg_id < num_images; msg_id++) {

    // Lock this data feed for this camera
    size_t cam_id = message.sensor_ids.at(msg_id);
    std::lock_guard<std::mutex> lck(mtx_feeds.at(cam_id));

    // Histogram equalize
    cv::Mat img;
    if (histogram_method == HistogramMethod::HISTOGRAM) {
      cv::equalizeHist(message.images.at(msg_id), img);
    } else if (histogram_method == HistogramMethod::CLAHE) {
      double eq_clip_limit = 10.0;
      cv::Size eq_win_size = cv::Size(8, 8);
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
      clahe->apply(message.images.at(msg_id), img);
    } else {
      img = message.images.at(msg_id);
    }

    // Extract image pyramid
    std::vector<cv::Mat> imgpyr;
    cv::buildOpticalFlowPyramid(img, imgpyr, win_size, pyr_levels);

    // Save!
    img_curr[cam_id] = img;
    img_pyramid_curr[cam_id] = imgpyr;
  }

  // Either call our stereo or monocular version
  // If we are doing binocular tracking, then we should parallize our tracking
  if (num_images == 1) {
    feed_monocular(message, 0);
  } else if (num_images == 2 && use_stereo) {
    feed_stereo(message, 0, 1);
  } else if (!use_stereo) {
    parallel_for_(cv::Range(0, (int)num_images), LambdaBody([&](const cv::Range &range) {
                    for (int i = range.start; i < range.end; i++) {
                      feed_monocular(message, i);
                    }
                  }));
  } else {
    PRINT_ERROR(RED "[ERROR]: invalid number of images passed %zu, we only support mono or stereo tracking", num_images);
    std::exit(EXIT_FAILURE);
  }
}

void TrackKLT::feed_monocular(const CameraData &message, size_t msg_id) {

  // Lock this data feed for this camera
  size_t cam_id = message.sensor_ids.at(msg_id);
  std::lock_guard<std::mutex> lck(mtx_feeds.at(cam_id));
  frame_count_++;

  // Get our image objects for this image
  cv::Mat img = img_curr.at(cam_id);
  std::vector<cv::Mat> imgpyr = img_pyramid_curr.at(cam_id);
  cv::Mat mask = message.masks.at(msg_id);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // If we didn't have any successful tracks last time, just extract this time
  // This also handles, the tracking initalization on the first call to this extractor
  if (pts_last[cam_id].empty()) {
    // Detect new features
    std::vector<cv::KeyPoint> good_left;
    std::vector<size_t> good_ids_left;
    perform_detection_monocular(imgpyr, mask, good_left, good_ids_left);
    // Save the current image and pyramid
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id] = img;
    img_pyramid_last[cam_id] = imgpyr;
    img_mask_last[cam_id] = mask;
    pts_last[cam_id] = good_left;
    ids_last[cam_id] = good_ids_left;
    return;
  }

  // First we should make that the last images have enough features so we can do KLT
  // This will "top-off" our number of tracks so always have a constant number
  int pts_before_detect = (int)pts_last[cam_id].size();
  auto pts_left_old = pts_last[cam_id];
  auto ids_left_old = ids_last[cam_id];
  // ids above this watermark are the candidates detected here, i.e. the ones the delayed branches may drop
  const size_t id_wm = currid;
  const SelectMode cmode0 = active_select_mode();
  const bool prob_mode = (cmode0 == SelectMode::PARALLAX || cmode0 == SelectMode::REPLACE);
  const bool replace_mode = (cmode0 == SelectMode::REPLACE);
  std::vector<cv::KeyPoint> promoted;
  const size_t prob_before_ = prob_.size();
  std::unordered_set<size_t> prob_ids;
  if (prob_mode) {
    // Probation points take part in the detector's occupancy grid and in its 15 px mask, but they are not
    // features.  Without this they would be invisible to the grid, every cell would look depleted, and the
    // detector would keep entering candidates -- the feature budget would silently grow (measured: 242
    // tracks and 50 births/frame against the baseline's 166 and 12.5).
    for (const auto &p : prob_)
      prob_ids.insert(p.id);
    for (const auto &p : prob_) {
      pts_left_old.emplace_back(p.pt, 0.f, 0.f);
      ids_left_old.push_back(p.id);
    }
  }
  perform_detection_monocular(img_pyramid_last[cam_id], img_mask_last[cam_id], pts_left_old, ids_left_old);
  rT3 = boost::posix_time::microsec_clock::local_time();

  // OV_SELECT=parallax: the detected candidates do not become features.  They enter a probation list
  // and are tracked for prob_k frames; only the ones whose displacement, minus the accumulated
  // frame-median displacement, reaches prob_theta are promoted to features.  That gate is the first
  // point in the pipeline where depth (parallax) is observable, which is what the survival statistics
  // cannot see at birth.
  if (prob_mode) {
    std::vector<cv::KeyPoint> old_pts;
    std::vector<size_t> old_ids;
    old_pts.reserve(pts_left_old.size());
    old_ids.reserve(pts_left_old.size());
    for (size_t k = 0; k < pts_left_old.size(); ++k) {
      if (ids_left_old[k] > id_wm)
        prob_.push_back({ids_left_old[k], pts_left_old[k].pt, pts_left_old[k].pt, med_path_, frame_count_});
      else if (prob_ids.find(ids_left_old[k]) == prob_ids.end()) {
        old_pts.push_back(pts_left_old[k]);
        old_ids.push_back(ids_left_old[k]);
      }
    }
    pts_left_old.swap(old_pts);
    ids_left_old.swap(old_ids);
    if (frame_count_ % 50 == 0)
      PRINT_INFO("[PROB] f=%zu entries=%zu prob=%zu\n", frame_count_, prob_.size() - prob_before_, prob_.size());
  }

  // Our return success masks, and predicted new features
  std::vector<uchar> mask_ll;
  std::vector<cv::KeyPoint> pts_left_new = pts_left_old;

  // Lets track temporally
  perform_matching(img_pyramid_last[cam_id], imgpyr, pts_left_old, pts_left_new, cam_id, cam_id, mask_ll);
  assert(pts_left_new.size() == ids_left_old.size());
  rT4 = boost::posix_time::microsec_clock::local_time();

  // If any of our mask is empty, that means we didn't have enough to do ransac, so just return
  if (mask_ll.empty()) {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id] = img;
    img_pyramid_last[cam_id] = imgpyr;
    img_mask_last[cam_id] = mask;
    pts_last[cam_id].clear();
    ids_last[cam_id].clear();
    PRINT_ERROR(RED "[KLT-EXTRACTOR]: Failed to get enough points to do RANSAC, resetting.....\n" RESET);
    return;
  }

  // Get our "good tracks"
  std::vector<cv::KeyPoint> good_left;
  std::vector<size_t> good_ids_left;
  std::vector<size_t> good_src;   // index into pts_left_new, for the delayed branches below

  // Loop through all left points
  for (size_t i = 0; i < pts_left_new.size(); i++) {
    // Ensure we do not have any bad KLT tracks (i.e., points are negative)
    if (pts_left_new.at(i).pt.x < 0 || pts_left_new.at(i).pt.y < 0 || (int)pts_left_new.at(i).pt.x >= img.cols ||
        (int)pts_left_new.at(i).pt.y >= img.rows)
      continue;
    // Check if it is in the mask
    // NOTE: mask has max value of 255 (white) if it should be
    if ((int)message.masks.at(msg_id).at<uint8_t>((int)pts_left_new.at(i).pt.y, (int)pts_left_new.at(i).pt.x) > 127)
      continue;
    // If it is a good track, and also tracked from left to right
    if (mask_ll[i]) {
      good_left.push_back(pts_left_new[i]);
      good_ids_left.push_back(ids_left_old[i]);
      good_src.push_back(i);
    }
  }

  // Probation: advance the frame-median path, track the probed candidates one frame and promote the ones
  // that have shown parallax.  The median is taken over the accepted tracks, so it carries the camera's
  // own motion (rotation and forward flow) and leaves the depth-dependent part in the residual.
  if (prob_mode) {
    static int dbg_eval = 0, dbg_pass = 0;
    static double dbg_motion = 0, dbg_e = 0;
    std::vector<int> cell_promoted((size_t)grid_x * grid_y, 0);
    // the same per-cell quota the detector uses (num_features_grid in perform_detection_monocular)
    const int quota = (int)((double)num_features / (double)(grid_x * grid_y)) + 1;
    std::vector<float> dx, dy;
    dx.reserve(pts_left_new.size());
    dy.reserve(pts_left_new.size());
    for (size_t i = 0; i < pts_left_new.size(); ++i) {
      if (!mask_ll[i])
        continue;
      dx.push_back(pts_left_new[i].pt.x - pts_left_old[i].pt.x);
      dy.push_back(pts_left_new[i].pt.y - pts_left_old[i].pt.y);
    }
    if (dx.size() > 10) {
      const size_t mid = dx.size() / 2;
      std::nth_element(dx.begin(), dx.begin() + mid, dx.end());
      std::nth_element(dy.begin(), dy.begin() + mid, dy.end());
      med_path_ += cv::Point2f(dx[mid], dy[mid]);
    }
    if (!prob_.empty()) {
      static const int prob_k = []() {
        const char *s = std::getenv("OV_PROB_K");
        return s ? std::atoi(s) : 10;
      }();
      static const float prob_theta = []() {
        const char *s = std::getenv("OV_PROB_THETA");
        return s ? (float)std::atof(s) : 3.0f;
      }();
      std::vector<cv::Point2f> in, out;
      std::vector<uchar> st;
      std::vector<float> err;
      in.reserve(prob_.size());
      for (const auto &p : prob_)
        in.push_back(p.pt);
      cv::TermCriteria tc(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
      cv::calcOpticalFlowPyrLK(img_pyramid_last[cam_id], imgpyr, in, out, st, err, win_size, pyr_levels, tc);
      std::vector<ProbPoint> keep;
      keep.reserve(prob_.size());
      struct PassedCand {
        float score;
        int cx, cy;
        cv::Point2f pt;
      };
      std::vector<PassedCand> passed;
      for (size_t k = 0; k < prob_.size(); ++k) {
        if (!st[k])
          continue;
        if (out[k].x < 0 || out[k].y < 0 || out[k].x >= imgpyr.at(0).cols || out[k].y >= imgpyr.at(0).rows)
          continue;
        ProbPoint p = prob_[k];
        p.pt = out[k];
        if (frame_count_ - p.birth_frame >= (size_t)prob_k) {
          const cv::Point2f motion = med_path_ - p.birth_m;
          const cv::Point2f e = (p.pt - p.birth_pt) - motion;
          // below the gate's own scale the parallax cannot be measured reliably (standstill, creep):
          // falling back to survival there keeps the frontend fed, and the initialiser working, without
          // changing what the gate means once the camera actually moves
          const bool measurable = std::hypot(motion.x, motion.y) >= prob_theta;
          dbg_eval++;
          dbg_motion += std::hypot(motion.x, motion.y);
          dbg_e += std::hypot(e.x, e.y);
          const bool pass = (!measurable || std::hypot(e.x, e.y) >= prob_theta);
          if (pass) {
            const int cpx = std::max(1, img.cols / grid_x);
            const int cpy = std::max(1, img.rows / grid_y);
            const int cx = std::min(grid_x - 1, std::max(0, (int)out[k].x / cpx));
            const int cy = std::min(grid_y - 1, std::max(0, (int)out[k].y / cpy));
            passed.push_back({(float)std::hypot(e.x, e.y), cx, cy, p.pt});
          }
        } else {
          keep.push_back(p);
        }
      }
      prob_.swap(keep);

      // Replace mode keeps the feature count where it is: only as many candidates are promoted as the
      // frame lost, best parallax first and at most one cell's quota each.  Parallax mode keeps the
      // per-cell quota instead (it is the gate, not the budget, that is under test there).
      int allow = (int)passed.size();
      if (replace_mode)
        allow = std::min(allow, std::max(0, pts_before_detect - (int)good_left.size()));
      std::sort(passed.begin(), passed.end(),
                [](const PassedCand &a, const PassedCand &b) { return a.score > b.score; });
      for (const auto &c : passed) {
        if ((int)promoted.size() >= allow)
          break;
        int &cnt = cell_promoted[(size_t)c.cy * grid_x + c.cx];
        if (cnt >= quota)
          continue;
        cnt++;
        dbg_pass++;
        promoted.emplace_back(c.pt, 0.f, 0.f);
      }
    }
    for (auto &kp : promoted) {
      good_left.push_back(kp);
      good_ids_left.push_back(currid++);
    }
    if (frame_count_ % 50 == 0)
      PRINT_INFO("[PROB] f=%zu kept=%zu promoted=%zu med=%.2f,%.2f eval=%d pass=%d motion=%.2f e=%.2f\n",
                 frame_count_, prob_.size(), promoted.size(), med_path_.x, med_path_.y, dbg_eval, dbg_pass,
                 dbg_eval ? dbg_motion / dbg_eval : 0.0, dbg_eval ? dbg_e / dbg_eval : 0.0);
    if (frame_count_ % 50 == 0) {
      dbg_eval = dbg_pass = 0;
      dbg_motion = dbg_e = 0;
    }
  }

  // Experiment C, delayed branches (OV_SELECT=cp1|cp3): a candidate's own first step is already finished
  // here, so its displacement (C-P1) or its forward-backward error (C-P3) can decide whether it survives to
  // become a birth.  The detection over-sampled 3x so that this filter does not shrink the feature budget:
  // exactly as many candidates are kept as the budget needs, chosen by the statistic.
  {
    const SelectMode cmode = active_select_mode();
    if ((cmode == SelectMode::C_P1 || cmode == SelectMode::C_P3) && !good_left.empty()) {
      const int n_old = (int)good_left.size() - (int)std::count_if(good_ids_left.begin(), good_ids_left.end(),
                                                                   [&](size_t id) { return id > id_wm; });
      const int n_keep = std::max(0, num_features - n_old);
      std::vector<size_t> cand;
      for (size_t i = 0; i < good_ids_left.size(); ++i)
        if (good_ids_left[i] > id_wm)
          cand.push_back(i);
      if ((int)cand.size() > n_keep) {
        std::vector<cv::Point2f> cur, back;
        cur.reserve(cand.size());
        for (size_t i : cand)
          cur.push_back(good_left[i].pt);
        std::vector<uchar> fbok;
        std::vector<float> fberr;
        if (cmode == SelectMode::C_P3) {
          cv::TermCriteria fbcrit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
          cv::calcOpticalFlowPyrLK(imgpyr, img_pyramid_last[cam_id], cur, back, fbok, fberr, win_size, pyr_levels, fbcrit);
        }
        std::vector<std::pair<double, size_t>> ranked;
        ranked.reserve(cand.size());
        for (size_t k = 0; k < cand.size(); ++k) {
          const cv::Point2f det = pts_left_new[good_src[cand[k]]].pt;   // where it was detected (frame t)
          double score = 0.0;
          if (cmode == SelectMode::C_P1) {
            const cv::Point2f d = good_left[cand[k]].pt - det;
            score = -std::hypot(d.x, d.y);                              // smaller first step survives better
          } else {
            const cv::Point2f e = (k < back.size()) ? (back[k] - det) : cv::Point2f(1e6f, 1e6f);
            score = -std::hypot(e.x, e.y);                              // smaller FB error survives better
          }
          ranked.emplace_back(score, cand[k]);
        }
        std::sort(ranked.begin(), ranked.end(),
                  [](const std::pair<double, size_t> &a, const std::pair<double, size_t> &b) { return a.first > b.first; });
        // Coverage first: every non-empty cell contributes its best candidate before any cell contributes a
        // second one.  A plain global top-k would let the score decide the spatial distribution too (for
        // C-P1 the smallest first step sits near the focus of expansion, i.e. on the farthest features, so
        // the births would collapse onto the centre and the comparison would measure that bias rather than
        // the statistic).  The grid is the constraint the baseline also obeys, so it is kept here.
        const int cell_px_x = img.cols / grid_x;
        const int cell_px_y = img.rows / grid_y;
        std::vector<std::vector<size_t>> bycell((size_t)grid_x * grid_y);
        for (const auto &kv : ranked) {
          const cv::Point2f &pt = good_left[kv.second].pt;
          const int cx = std::min(grid_x - 1, std::max(0, (int)pt.x / cell_px_x));
          const int cy = std::min(grid_y - 1, std::max(0, (int)pt.y / cell_px_y));
          bycell[(size_t)cy * grid_x + cx].push_back(kv.second);
        }
        std::vector<char> keep(good_left.size(), 0);
        int taken = 0;
        for (size_t r = 0; taken < n_keep; ++r) {
          bool any = false;
          for (const auto &cell : bycell) {
            if (r >= cell.size())
              continue;
            keep[cell[r]] = 1;
            taken++;
            any = true;
            if (taken >= n_keep)
              break;
          }
          if (!any)
            break;
        }
        std::vector<cv::KeyPoint> gl2;
        std::vector<size_t> gi2, gs2;
        for (size_t i = 0; i < good_left.size(); ++i) {
          if (good_ids_left[i] <= id_wm || keep[i]) {
            gl2.push_back(good_left[i]);
            gi2.push_back(good_ids_left[i]);
            gs2.push_back(good_src[i]);
          }
        }
        good_left.swap(gl2);
        good_ids_left.swap(gi2);
        good_src.swap(gs2);
      }
    }
  }

  // Update our feature database, with theses new observations
  if (frame_count_ % 50 == 0) {
    size_t n_births = 0;
    for (size_t id : good_ids_left)
      n_births += (id > id_wm) ? 1 : 0;
    const int lost = pts_before_detect - (int)good_left.size() + (int)n_births;
    PRINT_INFO("[KLT] f=%zu tracks=%zu births=%zu lost=%d\n", frame_count_, good_left.size(), n_births,
               std::max(0, lost));
  }
  for (size_t i = 0; i < good_left.size(); i++) {
    cv::Point2f npt_l = camera_calib.at(cam_id)->undistort_cv(good_left.at(i).pt);
    database->update_feature(good_ids_left.at(i), message.timestamp, cam_id, good_left.at(i).pt.x, good_left.at(i).pt.y, npt_l.x, npt_l.y);
  }

  // T3 bookkeeping (OV_WEIGHT=t3): features tracked last frame that are gone now, with their last
  // position.  A birth in a neighbourhood that has been losing tracks is a less reliable anchor.
  {
    std::unordered_set<size_t> alive(good_ids_left.begin(), good_ids_left.end());
    for (size_t i = 0; i < ids_last[cam_id].size(); ++i)
      if (alive.find(ids_last[cam_id][i]) == alive.end())
        deaths_.emplace_back(frame_count_, pts_last[cam_id][i].pt);
    while (!deaths_.empty() && frame_count_ - deaths_.front().first > 10)
      deaths_.pop_front();
  }

  // Experiment D: per-feature measurement-noise multiplier.  The frontend already knows something the
  // estimator does not: how well each point can be tracked.  OV_WEIGHT=l2 uses lambda2 at birth, whose
  // inverse is the KLT displacement variance (Cov(dp) = 2 sigma^2 M^-1); OV_WEIGHT=fb uses the
  // forward-backward error of the current track, a direct error estimate.  OV_WALPHA tempers the
  // lambda2 ratio (0.5 = sigma proportional to 1/lambda2, 0 = no weighting).
  {
    static const std::string wmode = []() {
      const char *s = std::getenv("OV_WEIGHT");
      return s ? std::string(s) : std::string();
    }();
    static const double walpha = []() {
      const char *s = std::getenv("OV_WALPHA");
      return s ? std::atof(s) : 0.5;
    }();
    // how far the per-feature sigma may deviate from the nominal one: the estimator is sensitive, so
    // the deviation is bounded on both sides (0.1 -> sigma in [0.9, 1.1] of nominal)
    static const double wclamp = []() {
      const char *s = std::getenv("OV_WCLAMP");
      const double v = s ? std::atof(s) : 0.2;
      return v > 0.0 ? v : 0.2;
    }();
    if ((wmode == "l2" || wmode == "l2init" || wmode == "l2cov" || wmode == "fb_l2") && !good_left.empty()) {
      CandStats st;
      prepare_stats(st, imgpyr, false);
      for (size_t i = 0; i < good_left.size(); ++i) {
        if (good_ids_left[i] <= id_wm)
          continue;   // a track keeps the quality it was born with
        double l1 = 0.0, l2v = 0.0;
        if (!tensor_eig(st.Ix, st.Iy, (int)std::lround(good_left[i].pt.x), (int)std::lround(good_left[i].pt.y), 7, l1, l2v))
          continue;
        auto f = database->get_feature(good_ids_left[i]);
        if (!f || l2v <= 1e-9)
          continue;
        l2_ref_ = (l2_ref_ <= 0.0) ? l2v : 0.995 * l2_ref_ + 0.005 * l2v;
        const double w = std::pow(l2_ref_ / l2v, 0.5 * walpha);
        // l2 effects both init and update; l2init is an initialization-only prior;
        // fb_l2 combines the per-frame FB multiplier with this init-only term.
        if (wmode == "l2")
          f->sigma_mult = std::min(1.0 + wclamp, std::max(1.0 - wclamp, w));
        else if (wmode == "l2cov") {
          Eigen::Matrix2d C;
          const double ref = (l2_ref_ > 0.0) ? l2_ref_ : l2v;
          if (tensor_cov(st.Ix, st.Iy, (int)std::lround(good_left[i].pt.x),
                         (int)std::lround(good_left[i].pt.y), 7, ref, C))
            f->init_cov = C;
        } else
          f->sigma_mult_init = std::min(1.0 + wclamp, std::max(1.0 - wclamp, w));
      }
    } else if ((wmode == "p3" || wmode == "p1" || wmode == "p1init") && !good_left.empty()) {
      // Birth-time step statistics, the same definitions as the offline P family: p3 is the
      // forward-backward error of the candidate's own first step (C-P3), p1 is its first-step
      // displacement relative to the frame's median (C-P1).  A feature keeps the prior it was born
      // with, exactly like the l2 mode.
      std::vector<size_t> idx;
      std::vector<cv::Point2f> cur, prev;
      for (size_t i = 0; i < good_left.size(); ++i) {
        if (good_ids_left[i] <= id_wm || i >= good_src.size())
          continue;
        idx.push_back(i);
        cur.push_back(good_left[i].pt);
        prev.push_back(pts_left_old[good_src[i]].pt);
      }
      if (!idx.empty()) {
        double d_med = 0.0;
        if (wmode == "p1" || wmode == "p1init") {
          // OV_P1_MEDIAN=old uses the old-track displacement median (the
          // previous behavior); the default is the median of the new births'
          // own first steps, which is the population being ranked.
          static const bool birth_median = []() {
            const char *s = std::getenv("OV_P1_MEDIAN");
            return !s || std::string(s) != "old";
          }();
          std::vector<float> ds;
          if (birth_median) {
            ds.reserve(idx.size());
            for (size_t k = 0; k < idx.size(); ++k)
              ds.push_back(cv::norm(cur[k] - prev[k]));
          } else {
            for (size_t i = 0; i < pts_left_new.size(); ++i)
              if (mask_ll[i] && ids_left_old[i] <= id_wm)
                ds.push_back(cv::norm(pts_left_new[i].pt - pts_left_old[i].pt));
          }
          if (!ds.empty()) {
            std::nth_element(ds.begin(), ds.begin() + ds.size() / 2, ds.end());
            d_med = ds[ds.size() / 2];
          }
        }
        std::vector<cv::Point2f> back;
        std::vector<uchar> stt;
        std::vector<float> err;
        if (wmode == "p3") {
          cv::TermCriteria tc(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
          cv::calcOpticalFlowPyrLK(imgpyr, img_pyramid_last[cam_id], cur, back, stt, err, win_size, pyr_levels, tc);
        }
        // p3 is used RELATIVE to this frame's median first-step FB error: with the absolute 1 px
        // reference almost every birth sits below it (the frontend's gate already rejected the bad
        // ones), the weight collapses to 1 and the prior is inert.  q50 is the frame's median over the
        // births, so "worse than the typical first step" is what gets down-weighted.
        std::vector<double> fbvals(idx.size(), 1.0);
        double q50 = 0.0;
        if (wmode == "p3") {
          std::vector<double> tmp;
          tmp.reserve(idx.size());
          for (size_t k = 0; k < idx.size(); ++k) {
            const double fb = (k < back.size() && stt[k]) ? cv::norm(back[k] - prev[k]) : 1e3;
            fbvals[k] = fb;
            if (std::isfinite(fb) && fb < 1e2)
              tmp.push_back(fb);
          }
          if (!tmp.empty()) {
            std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
            q50 = std::max(tmp[tmp.size() / 2], 1e-3);
          }
        }
        for (size_t k = 0; k < idx.size(); ++k) {
          auto f = database->get_feature(good_ids_left[idx[k]]);
          if (!f)
            continue;
          double w = 1.0;
          if (wmode == "p3") {
            w = (q50 > 0.0) ? std::pow(std::max(fbvals[k], q50) / q50, 0.5) : 1.0;
          } else if (d_med > 1e-3) {
            const double d = cv::norm(cur[k] - prev[k]);
            if (wmode == "p1init") {
              // Parallax/depth prior: low first-step displacement -> high initial depth
              // uncertainty.  Only inflate, and only at landmark initialization.
              w = 1.0 + walpha * std::max(0.0, d_med / std::max(d, 0.5) - 1.0);
              f->sigma_mult_init = std::min(2.0, std::max(1.0, w));
              continue;
            }
            w = d_med / std::max(d, 0.5);   // little motion means far away, where depth is poorly observed
          }
          f->sigma_mult = std::min(1.0 + wclamp, std::max(1.0 - wclamp, w));
        }
      }
    } else if ((wmode == "fst" || wmode == "harris" || wmode == "csr" || wmode == "cscale" ||
                wmode == "round" || wmode == "ent") &&
               !good_left.empty()) {
      // The other A/B statistics as a measurement-precision prior.  The semantics here is "higher
      // quality = more precise bearing", which is NOT the same as the offline survival direction
      // (CSR, for instance, survives better when small but is sharper when large).
      CandStats st;
      prepare_stats(st, imgpyr, wmode == "cscale");
      static double q_ref = 0.0;
      const int cell_px_x2 = std::max(1, (int)imgpyr.at(0).cols / grid_x);
      (void)cell_px_x2;
      for (size_t i = 0; i < good_left.size(); ++i) {
        if (good_ids_left[i] <= id_wm)
          continue;
        const int x = (int)std::lround(good_left[i].pt.x);
        const int y = (int)std::lround(good_left[i].pt.y);
        double l1 = 0.0, l2v = 0.0, q = 0.0;
        if (wmode == "fst" || wmode == "harris") {
          if (!tensor_eig(st.Ix, st.Iy, x, y, 7, l1, l2v))
            continue;
          q = (wmode == "fst") ? (l1 * l2v) / std::max(l1 + l2v, 1e-12)
                               : (l1 * l2v - 0.04 * (l1 + l2v) * (l1 + l2v));
        } else if (wmode == "csr") {
          if (!tensor_eig(st.Ix, st.Iy, x, y, 7, l1, l2v))
            continue;
          double d1 = 0.0, d2 = 0.0;
          if (!tensor_eig(st.hIx, st.hIy, x / 2, y / 2, 3, d1, d2))
            continue;
          q = d2 / std::max(l2v, 1e-12);   // fine-scale dominance: sharper = more precise
        } else if (wmode == "round") {
          if (!tensor_eig(st.Ix, st.Iy, x, y, 7, l1, l2v))
            continue;
          q = l2v / std::max(l1, 1e-12);   // lambda2/lambda1: isotropic corner = well localized
        } else if (wmode == "ent") {
          const cv::Rect roi(x - 7, y - 7, 15, 15);
          if (roi.x < 0 || roi.y < 0 || roi.x + roi.width > imgpyr.at(0).cols || roi.y + roi.height > imgpyr.at(0).rows)
            continue;
          int hist[32] = {0};
          const cv::Mat patch = imgpyr.at(0)(roi);
          for (int r = 0; r < patch.rows; ++r)
            for (int c = 0; c < patch.cols; ++c)
              hist[patch.at<uchar>(r, c) * 32 / 256]++;
          double H = 0.0;
          for (int b = 0; b < 32; ++b)
            if (hist[b] > 0) {
              const double pp = (double)hist[b] / 225.0;
              H -= pp * std::log(pp);
            }
          q = H / std::log(32.0);   // normalized texture entropy
        } else {
          // characteristic scale: argmax sigma * l2; a smaller scale is a sharper corner
          double best_sig = cscale_sigmas().back();
          double bestv = -1.0;
          bool ok = true;
          for (size_t k = 0; k < st.bIx.size(); ++k) {
            if (!tensor_eig(st.bIx[k], st.bIy[k], x, y, 7, l1, l2v)) {
              ok = false;
              break;
            }
            const double v = cscale_sigmas()[k] * l2v;
            if (v > bestv) {
              bestv = v;
              best_sig = cscale_sigmas()[k];
            }
          }
          if (!ok)
            continue;
          q = 1.0 / std::max(best_sig, 1e-3);
        }
        if (q <= 0.0)
          continue;
        q_ref = (q_ref <= 0.0) ? q : 0.995 * q_ref + 0.005 * q;
        const double w = std::pow(q_ref / q, 0.5 * walpha);
        auto f = database->get_feature(good_ids_left[i]);
        if (f)
          f->sigma_mult = std::min(1.0 + wclamp, std::max(1.0 - wclamp, w));
      }
    } else if (wmode == "t3" && !good_left.empty()) {
      // T3: recent deaths in a 30 px neighbourhood over the last 10 frames.  More deaths means the
      // area has been unstable (dynamic objects, degraded texture), so the birth is less trustworthy.
      for (size_t i = 0; i < good_left.size(); ++i) {
        if (good_ids_left[i] <= id_wm)
          continue;
        int nd = 0;
        for (const auto &d : deaths_)
          if (cv::norm(d.second - good_left[i].pt) <= 30.0)
            nd++;
        const double w = 1.0 + wclamp * std::min(1.0, nd / 3.0);
        auto f = database->get_feature(good_ids_left[i]);
        if (f)
          f->sigma_mult = std::min(1.0 + wclamp, std::max(1.0 - wclamp, w));
      }
    } else if (wmode == "fb" && !good_left.empty()) {
      std::vector<cv::Point2f> cur, back;
      cur.reserve(good_left.size());
      for (const auto &k : good_left)
        cur.push_back(k.pt);
      std::vector<uchar> stt;
      std::vector<float> err;
      cv::TermCriteria tc(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
      cv::calcOpticalFlowPyrLK(imgpyr, img_pyramid_last[cam_id], cur, back, stt, err, win_size, pyr_levels, tc);
      for (size_t i = 0; i < good_left.size() && i < back.size(); ++i) {
        auto f = database->get_feature(good_ids_left[i]);
        if (!f)
          continue;
        const cv::Point2f prev = (i < good_src.size()) ? pts_left_old[good_src[i]].pt : good_left[i].pt;
        const double fb = stt[i] ? cv::norm(back[i] - prev) : 1e3;
        // one pixel of FB error is the nominal track; worse tracks get a larger sigma, never smaller
        f->sigma_mult = std::min(1.0 + wclamp, std::max(1.0, fb));
      }
    }
    // Per-frame relative FB error: R/R0 = (1 + alpha * max(0, e_i/e_med - 1))^2.
    // Only tracks worse than the frame median are down-weighted; good tracks stay
    // at the nominal variance.  fb_l2 also carries the l2init prior, set above.
    if ((wmode == "fb_rel" || wmode == "fb_l2") && !good_left.empty()) {
      std::vector<cv::Point2f> cur, back;
      cur.reserve(good_left.size());
      for (const auto &k : good_left)
        cur.push_back(k.pt);
      std::vector<uchar> stt;
      std::vector<float> err;
      cv::TermCriteria tc(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
      cv::calcOpticalFlowPyrLK(imgpyr, img_pyramid_last[cam_id], cur, back, stt, err, win_size, pyr_levels, tc);
      std::vector<double> fb(good_left.size(), 1e3), tmp;
      tmp.reserve(good_left.size());
      for (size_t i = 0; i < good_left.size() && i < back.size(); ++i) {
        const cv::Point2f prev = (i < good_src.size()) ? pts_left_old[good_src[i]].pt : good_left[i].pt;
        if (stt[i]) {
          fb[i] = cv::norm(back[i] - prev);
          if (std::isfinite(fb[i]) && fb[i] < 1e3)
            tmp.push_back(fb[i]);
        }
      }
      double med = 1.0;
      if (!tmp.empty()) {
        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
        med = std::max(tmp[tmp.size() / 2], 1e-3);
      }
      for (size_t i = 0; i < good_left.size(); ++i) {
        auto f = database->get_feature(good_ids_left[i]);
        if (!f)
          continue;
        const double s = fb[i] / med;
        const double w = 1.0 + walpha * std::max(0.0, s - 1.0);
        f->sigma_mult = std::min(2.0, std::max(0.5, w));
      }
    }
    if (!wmode.empty() && frame_count_ % 50 == 0) {
      std::vector<double> ms;
      ms.reserve(good_ids_left.size());
      for (size_t id : good_ids_left) {
        auto f = database->get_feature(id);
        if (f)
          ms.push_back(f->sigma_mult);
      }
      if (!ms.empty()) {
        std::sort(ms.begin(), ms.end());
        PRINT_INFO("[WEIGHT] f=%zu mult median %.2f  p10 %.2f  p90 %.2f\n", frame_count_, ms[ms.size() / 2],
                   ms[ms.size() / 10], ms[ms.size() * 9 / 10]);
      }
    }
  }

  // Move forward in time
  {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id] = img;
    img_pyramid_last[cam_id] = imgpyr;
    img_mask_last[cam_id] = mask;
    pts_last[cam_id] = good_left;
    ids_last[cam_id] = good_ids_left;
  }
  rT5 = boost::posix_time::microsec_clock::local_time();

  // Timing information
  PRINT_ALL("[TIME-KLT]: %.4f seconds for pyramid\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[TIME-KLT]: %.4f seconds for detection (%zu detected)\n", (rT3 - rT2).total_microseconds() * 1e-6,
            (int)pts_last[cam_id].size() - pts_before_detect);
  PRINT_ALL("[TIME-KLT]: %.4f seconds for temporal klt\n", (rT4 - rT3).total_microseconds() * 1e-6);
  PRINT_ALL("[TIME-KLT]: %.4f seconds for feature DB update (%d features)\n", (rT5 - rT4).total_microseconds() * 1e-6,
            (int)good_left.size());
  PRINT_ALL("[TIME-KLT]: %.4f seconds for total\n", (rT5 - rT1).total_microseconds() * 1e-6);
}

void TrackKLT::feed_stereo(const CameraData &message, size_t msg_id_left, size_t msg_id_right) {

  // Lock this data feed for this camera
  size_t cam_id_left = message.sensor_ids.at(msg_id_left);
  size_t cam_id_right = message.sensor_ids.at(msg_id_right);
  std::lock_guard<std::mutex> lck1(mtx_feeds.at(cam_id_left));
  std::lock_guard<std::mutex> lck2(mtx_feeds.at(cam_id_right));

  // Get our image objects for this image
  cv::Mat img_left = img_curr.at(cam_id_left);
  cv::Mat img_right = img_curr.at(cam_id_right);
  std::vector<cv::Mat> imgpyr_left = img_pyramid_curr.at(cam_id_left);
  std::vector<cv::Mat> imgpyr_right = img_pyramid_curr.at(cam_id_right);
  cv::Mat mask_left = message.masks.at(msg_id_left);
  cv::Mat mask_right = message.masks.at(msg_id_right);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // If we didn't have any successful tracks last time, just extract this time
  // This also handles, the tracking initalization on the first call to this extractor
  if (pts_last[cam_id_left].empty() && pts_last[cam_id_right].empty()) {
    // Track into the new image
    std::vector<cv::KeyPoint> good_left, good_right;
    std::vector<size_t> good_ids_left, good_ids_right;
    perform_detection_stereo(imgpyr_left, imgpyr_right, mask_left, mask_right, cam_id_left, cam_id_right, good_left, good_right,
                             good_ids_left, good_ids_right);
    // Save the current image and pyramid
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id_left] = img_left;
    img_last[cam_id_right] = img_right;
    img_pyramid_last[cam_id_left] = imgpyr_left;
    img_pyramid_last[cam_id_right] = imgpyr_right;
    img_mask_last[cam_id_left] = mask_left;
    img_mask_last[cam_id_right] = mask_right;
    pts_last[cam_id_left] = good_left;
    pts_last[cam_id_right] = good_right;
    ids_last[cam_id_left] = good_ids_left;
    ids_last[cam_id_right] = good_ids_right;
    return;
  }

  // First we should make that the last images have enough features so we can do KLT
  // This will "top-off" our number of tracks so always have a constant number
  int pts_before_detect = (int)pts_last[cam_id_left].size();
  auto pts_left_old = pts_last[cam_id_left];
  auto pts_right_old = pts_last[cam_id_right];
  auto ids_left_old = ids_last[cam_id_left];
  auto ids_right_old = ids_last[cam_id_right];
  perform_detection_stereo(img_pyramid_last[cam_id_left], img_pyramid_last[cam_id_right], img_mask_last[cam_id_left],
                           img_mask_last[cam_id_right], cam_id_left, cam_id_right, pts_left_old, pts_right_old, ids_left_old,
                           ids_right_old);
  rT3 = boost::posix_time::microsec_clock::local_time();

  // Our return success masks, and predicted new features
  std::vector<uchar> mask_ll, mask_rr;
  std::vector<cv::KeyPoint> pts_left_new = pts_left_old;
  std::vector<cv::KeyPoint> pts_right_new = pts_right_old;

  // Lets track temporally
  parallel_for_(cv::Range(0, 2), LambdaBody([&](const cv::Range &range) {
                  for (int i = range.start; i < range.end; i++) {
                    bool is_left = (i == 0);
                    perform_matching(img_pyramid_last[is_left ? cam_id_left : cam_id_right], is_left ? imgpyr_left : imgpyr_right,
                                     is_left ? pts_left_old : pts_right_old, is_left ? pts_left_new : pts_right_new,
                                     is_left ? cam_id_left : cam_id_right, is_left ? cam_id_left : cam_id_right,
                                     is_left ? mask_ll : mask_rr);
                  }
                }));
  rT4 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  //===================================================================================

  // left to right matching
  // TODO: we should probably still do this to reject outliers
  // TODO: maybe we should collect all tracks that are in both frames and make they pass this?
  // std::vector<uchar> mask_lr;
  // perform_matching(imgpyr_left, imgpyr_right, pts_left_new, pts_right_new, cam_id_left, cam_id_right, mask_lr);
  rT5 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  //===================================================================================

  // If any of our masks are empty, that means we didn't have enough to do ransac, so just return
  if (mask_ll.empty() && mask_rr.empty()) {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id_left] = img_left;
    img_last[cam_id_right] = img_right;
    img_pyramid_last[cam_id_left] = imgpyr_left;
    img_pyramid_last[cam_id_right] = imgpyr_right;
    img_mask_last[cam_id_left] = mask_left;
    img_mask_last[cam_id_right] = mask_right;
    pts_last[cam_id_left].clear();
    pts_last[cam_id_right].clear();
    ids_last[cam_id_left].clear();
    ids_last[cam_id_right].clear();
    PRINT_ERROR(RED "[KLT-EXTRACTOR]: Failed to get enough points to do RANSAC, resetting.....\n" RESET);
    return;
  }

  // Get our "good tracks"
  std::vector<cv::KeyPoint> good_left, good_right;
  std::vector<size_t> good_ids_left, good_ids_right;

  // Loop through all left points
  for (size_t i = 0; i < pts_left_new.size(); i++) {
    // Ensure we do not have any bad KLT tracks (i.e., points are negative)
    if (pts_left_new.at(i).pt.x < 0 || pts_left_new.at(i).pt.y < 0 || (int)pts_left_new.at(i).pt.x > img_left.cols ||
        (int)pts_left_new.at(i).pt.y > img_left.rows)
      continue;
    // See if we have the same feature in the right
    bool found_right = false;
    size_t index_right = 0;
    for (size_t n = 0; n < ids_right_old.size(); n++) {
      if (ids_left_old.at(i) == ids_right_old.at(n)) {
        found_right = true;
        index_right = n;
        break;
      }
    }
    // If it is a good track, and also tracked from left to right
    // Else track it as a mono feature in just the left image
    if (mask_ll[i] && found_right && mask_rr[index_right]) {
      // Ensure we do not have any bad KLT tracks (i.e., points are negative)
      if (pts_right_new.at(index_right).pt.x < 0 || pts_right_new.at(index_right).pt.y < 0 ||
          (int)pts_right_new.at(index_right).pt.x >= img_right.cols || (int)pts_right_new.at(index_right).pt.y >= img_right.rows)
        continue;
      good_left.push_back(pts_left_new.at(i));
      good_right.push_back(pts_right_new.at(index_right));
      good_ids_left.push_back(ids_left_old.at(i));
      good_ids_right.push_back(ids_right_old.at(index_right));
      // PRINT_DEBUG("adding to stereo - %u , %u\n", ids_left_old.at(i), ids_right_old.at(index_right));
    } else if (mask_ll[i]) {
      good_left.push_back(pts_left_new.at(i));
      good_ids_left.push_back(ids_left_old.at(i));
      // PRINT_DEBUG("adding to left - %u \n",ids_left_old.at(i));
    }
  }

  // Loop through all right points
  for (size_t i = 0; i < pts_right_new.size(); i++) {
    // Ensure we do not have any bad KLT tracks (i.e., points are negative)
    if (pts_right_new.at(i).pt.x < 0 || pts_right_new.at(i).pt.y < 0 || (int)pts_right_new.at(i).pt.x >= img_right.cols ||
        (int)pts_right_new.at(i).pt.y >= img_right.rows)
      continue;
    // See if we have the same feature in the right
    bool added_already = (std::find(good_ids_right.begin(), good_ids_right.end(), ids_right_old.at(i)) != good_ids_right.end());
    // If it has not already been added as a good feature, add it as a mono track
    if (mask_rr[i] && !added_already) {
      good_right.push_back(pts_right_new.at(i));
      good_ids_right.push_back(ids_right_old.at(i));
      // PRINT_DEBUG("adding to right - %u \n", ids_right_old.at(i));
    }
  }

  // Update our feature database, with theses new observations
  for (size_t i = 0; i < good_left.size(); i++) {
    cv::Point2f npt_l = camera_calib.at(cam_id_left)->undistort_cv(good_left.at(i).pt);
    database->update_feature(good_ids_left.at(i), message.timestamp, cam_id_left, good_left.at(i).pt.x, good_left.at(i).pt.y, npt_l.x,
                             npt_l.y);
  }
  for (size_t i = 0; i < good_right.size(); i++) {
    cv::Point2f npt_r = camera_calib.at(cam_id_right)->undistort_cv(good_right.at(i).pt);
    database->update_feature(good_ids_right.at(i), message.timestamp, cam_id_right, good_right.at(i).pt.x, good_right.at(i).pt.y, npt_r.x,
                             npt_r.y);
  }

  // Move forward in time
  {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    img_last[cam_id_left] = img_left;
    img_last[cam_id_right] = img_right;
    img_pyramid_last[cam_id_left] = imgpyr_left;
    img_pyramid_last[cam_id_right] = imgpyr_right;
    img_mask_last[cam_id_left] = mask_left;
    img_mask_last[cam_id_right] = mask_right;
    pts_last[cam_id_left] = good_left;
    pts_last[cam_id_right] = good_right;
    ids_last[cam_id_left] = good_ids_left;
    ids_last[cam_id_right] = good_ids_right;
  }
  rT6 = boost::posix_time::microsec_clock::local_time();

  //  // Timing information
  PRINT_ALL("[TIME-KLT]: %.4f seconds for pyramid\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[TIME-KLT]: %.4f seconds for detection (%d detected)\n", (rT3 - rT2).total_microseconds() * 1e-6,
            (int)pts_last[cam_id_left].size() - pts_before_detect);
  PRINT_ALL("[TIME-KLT]: %.4f seconds for temporal klt\n", (rT4 - rT3).total_microseconds() * 1e-6);
  PRINT_ALL("[TIME-KLT]: %.4f seconds for stereo klt\n", (rT5 - rT4).total_microseconds() * 1e-6);
  PRINT_ALL("[TIME-KLT]: %.4f seconds for feature DB update (%d features)\n", (rT6 - rT5).total_microseconds() * 1e-6,
            (int)good_left.size());
  PRINT_ALL("[TIME-KLT]: %.4f seconds for total\n", (rT6 - rT1).total_microseconds() * 1e-6);
}

void TrackKLT::perform_detection_monocular(const std::vector<cv::Mat> &img0pyr, const cv::Mat &mask0, std::vector<cv::KeyPoint> &pts0,
                                           std::vector<size_t> &ids0) {

  // Create a 2D occupancy grid for this current image
  // Note that we scale this down, so that each grid point is equal to a set of pixels
  // This means that we will reject points that less than grid_px_size points away then existing features
  cv::Size size_close((int)((float)img0pyr.at(0).cols / (float)min_px_dist),
                      (int)((float)img0pyr.at(0).rows / (float)min_px_dist)); // width x height
  cv::Mat grid_2d_close = cv::Mat::zeros(size_close, CV_8UC1);
  float size_x = (float)img0pyr.at(0).cols / (float)grid_x;
  float size_y = (float)img0pyr.at(0).rows / (float)grid_y;
  cv::Size size_grid(grid_x, grid_y); // width x height
  cv::Mat grid_2d_grid = cv::Mat::zeros(size_grid, CV_8UC1);
  cv::Mat mask0_updated = mask0.clone();
  auto it0 = pts0.begin();
  auto it1 = ids0.begin();
  while (it0 != pts0.end()) {
    // Get current left keypoint, check that it is in bounds
    cv::KeyPoint kpt = *it0;
    int x = (int)kpt.pt.x;
    int y = (int)kpt.pt.y;
    int edge = 10;
    if (x < edge || x >= img0pyr.at(0).cols - edge || y < edge || y >= img0pyr.at(0).rows - edge) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Calculate mask coordinates for close points
    int x_close = (int)(kpt.pt.x / (float)min_px_dist);
    int y_close = (int)(kpt.pt.y / (float)min_px_dist);
    if (x_close < 0 || x_close >= size_close.width || y_close < 0 || y_close >= size_close.height) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Calculate what grid cell this feature is in
    int x_grid = std::floor(kpt.pt.x / size_x);
    int y_grid = std::floor(kpt.pt.y / size_y);
    if (x_grid < 0 || x_grid >= size_grid.width || y_grid < 0 || y_grid >= size_grid.height) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Check if this keypoint is near another point
    if (grid_2d_close.at<uint8_t>(y_close, x_close) > 127) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Now check if it is in a mask area or not
    // NOTE: mask has max value of 255 (white) if it should be
    if (mask0.at<uint8_t>(y, x) > 127) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Else we are good, move forward to the next point
    grid_2d_close.at<uint8_t>(y_close, x_close) = 255;
    if (grid_2d_grid.at<uint8_t>(y_grid, x_grid) < 255) {
      grid_2d_grid.at<uint8_t>(y_grid, x_grid) += 1;
    }
    // Append this to the local mask of the image
    if (x - min_px_dist >= 0 && x + min_px_dist < img0pyr.at(0).cols && y - min_px_dist >= 0 && y + min_px_dist < img0pyr.at(0).rows) {
      cv::Point pt1(x - min_px_dist, y - min_px_dist);
      cv::Point pt2(x + min_px_dist, y + min_px_dist);
      cv::rectangle(mask0_updated, pt1, pt2, cv::Scalar(255), -1);
    }
    it0++;
    it1++;
  }

  // First compute how many more features we need to extract from this image
  // If we don't need any features, just return
  double min_feat_percent = 0.50;
  int num_featsneeded = num_features - (int)pts0.size();
  if (num_featsneeded < std::min(20, (int)(min_feat_percent * num_features)))
    return;

  // This is old extraction code that would extract from the whole image
  // This can be slow as this will recompute extractions for grid areas that we have max features already
  // std::vector<cv::KeyPoint> pts0_ext;
  // Grider_FAST::perform_griding(img0pyr.at(0), mask0_updated, pts0_ext, num_features, grid_x, grid_y, threshold, true);

  // We also check a downsampled mask such that we don't extract in areas where it is all masked!
  cv::Mat mask0_grid;
  cv::resize(mask0, mask0_grid, size_grid, 0.0, 0.0, cv::INTER_NEAREST);

  // Create grids we need to extract from and then extract our features (use fast with griding)
  int num_features_grid = (int)((double)num_features / (double)(grid_x * grid_y)) + 1;
  int num_features_grid_req = std::max(1, (int)(min_feat_percent * num_features_grid));
  std::vector<std::pair<int, int>> valid_locs;
  for (int x = 0; x < grid_2d_grid.cols; x++) {
    for (int y = 0; y < grid_2d_grid.rows; y++) {
      if ((int)grid_2d_grid.at<uint8_t>(y, x) < num_features_grid_req && (int)mask0_grid.at<uint8_t>(y, x) != 255) {
        valid_locs.emplace_back(x, y);
      }
    }
  }
  std::vector<cv::KeyPoint> pts0_ext;
  const SelectMode mode = active_select_mode();
  // 3x over-sampling in every non-baseline branch, so that the cell's choice is not a FAST preselection
  // and the branches differ only in the ranking.  OV_POOL raises it for the wide-band experiment.
  static const int pool_mult = []() {
    const char *s = std::getenv("OV_POOL");
    const int v = s ? std::atoi(s) : 3;
    return v > 0 ? v : 3;
  }();
  const int oversample = (mode == SelectMode::FAST) ? 1 : pool_mult;
  Grider_GRID::perform_griding(img0pyr.at(0), mask0_updated, valid_locs, pts0_ext, num_features * oversample, grid_x, grid_y,
                               threshold, true);

  // Experiment C: inside each grid cell keep the candidates with the best statistic.  The cell's cap
  // (num_features_grid) is unchanged, so the per-frame/per-cell coverage constraint is exactly the one
  // the baseline has -- only which candidates fill the cells changes.
  //
  // The minimum-distance rule has to be enforced DURING the selection, not after it: the highest-scoring
  // candidates of a cell tend to sit on the same corner, so picking the top K and only then rejecting
  // the ones closer than min_px_dist throws most of them away and quietly shrinks the feature budget.
  // (That is what the first version did: measured on fs_full, the classic single-frame rankings scored
  // 16-18 m against the baseline's 10.5 m, while the random branch -- whose scores are spread out, so
  // the rule rarely binds -- matched the baseline.  The budget loss, not the statistic, was being
  // measured.)
  const bool rerank = mode == SelectMode::FASTPOOL || mode == SelectMode::L2 || mode == SelectMode::FOSTNER ||
                      mode == SelectMode::CSR || mode == SelectMode::CSCALE || mode == SelectMode::RANDOM ||
                      mode == SelectMode::PARALLAX;
  if (rerank && !pts0_ext.empty()) {
    CandStats st;
    // FASTPOOL is the control for the enlarged pool: the same 3x oversampling and the same selection
    // code, ranked by the FAST response -- i.e. exactly what the baseline picks, from one pool.  If its
    // ATE differs from the baseline, the pool (not the statistic) is what changed.
    if (mode != SelectMode::FASTPOOL)
      prepare_stats(st, img0pyr, mode == SelectMode::CSCALE);
    // probation mode ranks by the response too; its difference is the delayed parallax gate, not the ranking
    const bool by_response = (mode == SelectMode::FASTPOOL || mode == SelectMode::PARALLAX);
    // a probation entry is a candidate, not a feature: keep twice the quota so that the ~40% of points
    // that pass the gate still fill the cell.  The bootstrap detection (no tracks yet, the frame the
    // initialiser starts from) keeps the baseline's quota: a wider set there only feeds the initialiser
    // weaker corners, and that is a different experiment.
    static const float prob_mult = []() {
      const char *s = std::getenv("OV_PROB_X");
      return s ? (float)std::atof(s) : 2.0f;
    }();
    const bool bootstrap = pts0.empty();
    const int per_cell = (mode == SelectMode::PARALLAX && !bootstrap)
                             ? (int)(num_features_grid * prob_mult)
                             : num_features_grid;
    std::vector<std::vector<std::pair<double, int>>> cells((size_t)grid_x * grid_y);
    // Same cell size as Grider_GRID, which uses integer division: a float cell edge would put the
    // candidates of the rightmost fraction of a cell into the neighbouring cell's bucket.
    const int cell_px_x = img0pyr.at(0).cols / grid_x;
    const int cell_px_y = img0pyr.at(0).rows / grid_y;
    for (size_t i = 0; i < pts0_ext.size(); ++i) {
      int cx = std::min(grid_x - 1, std::max(0, (int)pts0_ext[i].pt.x / cell_px_x));
      int cy = std::min(grid_y - 1, std::max(0, (int)pts0_ext[i].pt.y / cell_px_y));
      const uint32_t salt = (uint32_t)(i * 2654435761u) ^ (uint32_t)(currid & 0xffffu);
      const double sc =
          by_response
              ? (double)pts0_ext[i].response
              : cand_score(mode, st, (int)std::lround(pts0_ext[i].pt.x), (int)std::lround(pts0_ext[i].pt.y), salt);
      cells[(size_t)cy * grid_x + cx].emplace_back(sc, (int)i);
      // OV_DUMP_POOL=<file>: every candidate with BOTH rankings, so the disagreement between the
      // baseline's response order and the statistic can be measured on one and the same pool.
      if (!by_response) {
        static FILE *pool = nullptr;
        static bool pool_tried = false;
        if (!pool_tried) {
          pool_tried = true;
          const char *p = std::getenv("OV_DUMP_POOL");
          if (p)
            pool = std::fopen(p, "a");
        }
        if (pool)
          std::fprintf(pool, "%zu %d %d %.2f %.2f %d %.6g\n", frame_count_, cx, cy, pts0_ext[i].pt.x,
                       pts0_ext[i].pt.y, (int)std::lround(pts0_ext[i].response), sc);
      }
    }
    std::vector<cv::KeyPoint> kept;
    kept.reserve(pts0_ext.size());
    // The selection enforces the spacing on the live mask (so it accumulates across cells, exactly like
    // the outer loop), but the outer loop re-checks and would skip anything already marked.  Restore the
    // mask afterwards: the kept set is already spaced, so the outer loop accepts it unchanged.
    const cv::Mat close_before = grid_2d_close.clone();
    for (auto &cell : cells) {
      std::sort(cell.begin(), cell.end(),
                [](const std::pair<double, int> &a, const std::pair<double, int> &b) { return a.first > b.first; });
      int taken = 0;
      for (size_t k = 0; k < cell.size() && taken < per_cell; ++k) {
        const cv::KeyPoint &kp = pts0_ext[cell[k].second];
        const int x_close = (int)(kp.pt.x / (float)min_px_dist);
        const int y_close = (int)(kp.pt.y / (float)min_px_dist);
        if (x_close < 0 || x_close >= size_close.width || y_close < 0 || y_close >= size_close.height)
          continue;
        if (grid_2d_close.at<uint8_t>(y_close, x_close) > 127)
          continue;
        grid_2d_close.at<uint8_t>(y_close, x_close) = 255;
        kept.push_back(kp);
        taken++;
      }
    }
    close_before.copyTo(grid_2d_close);
    pts0_ext.swap(kept);
  }

  // Now, reject features that are close a current feature
  std::vector<cv::KeyPoint> kpts0_new;
  std::vector<cv::Point2f> pts0_new;
  for (auto &kpt : pts0_ext) {
    // Check that it is in bounds
    int x_grid = (int)(kpt.pt.x / (float)min_px_dist);
    int y_grid = (int)(kpt.pt.y / (float)min_px_dist);
    if (x_grid < 0 || x_grid >= size_close.width || y_grid < 0 || y_grid >= size_close.height)
      continue;
    // See if there is a point at this location
    if (grid_2d_close.at<uint8_t>(y_grid, x_grid) > 127)
      continue;
    // Else lets add it!
    kpts0_new.push_back(kpt);
    pts0_new.push_back(kpt.pt);
    grid_2d_close.at<uint8_t>(y_grid, x_grid) = 255;
  }

  // Optional birth dump (OV_DUMP=<file>): frame, x, y, FAST response.  Used to measure how much of
  // the birth set a selection rule actually changes; off unless the variable is set.
  {
    static FILE *dump = nullptr;
    static bool dump_tried = false;
    if (!dump_tried) {
      dump_tried = true;
      const char *p = std::getenv("OV_DUMP");
      if (p)
        dump = std::fopen(p, "a");
    }
    if (dump)
      for (const auto &k : kpts0_new)
        std::fprintf(dump, "%zu %.2f %.2f %.1f\n", frame_count_, k.pt.x, k.pt.y, k.response);
  }

  // Loop through and record only ones that are valid
  // NOTE: if we multi-thread this atomic can cause some randomness due to multiple thread detecting features
  // NOTE: this is due to the fact that we select update features based on feat id
  // NOTE: thus the order will matter since we try to select oldest (smallest id) to update with
  // NOTE: not sure how to remove... maybe a better way?
  for (size_t i = 0; i < pts0_new.size(); i++) {
    // update the uv coordinates
    kpts0_new.at(i).pt = pts0_new.at(i);
    // append the new uv coordinate
    pts0.push_back(kpts0_new.at(i));
    // move id foward and append this new point
    size_t temp = ++currid;
    ids0.push_back(temp);
  }
}

void TrackKLT::perform_detection_stereo(const std::vector<cv::Mat> &img0pyr, const std::vector<cv::Mat> &img1pyr, const cv::Mat &mask0,
                                        const cv::Mat &mask1, size_t cam_id_left, size_t cam_id_right, std::vector<cv::KeyPoint> &pts0,
                                        std::vector<cv::KeyPoint> &pts1, std::vector<size_t> &ids0, std::vector<size_t> &ids1) {

  // Create a 2D occupancy grid for this current image
  // Note that we scale this down, so that each grid point is equal to a set of pixels
  // This means that we will reject points that less then grid_px_size points away then existing features
  cv::Size size_close0((int)((float)img0pyr.at(0).cols / (float)min_px_dist),
                       (int)((float)img0pyr.at(0).rows / (float)min_px_dist)); // width x height
  cv::Mat grid_2d_close0 = cv::Mat::zeros(size_close0, CV_8UC1);
  float size_x0 = (float)img0pyr.at(0).cols / (float)grid_x;
  float size_y0 = (float)img0pyr.at(0).rows / (float)grid_y;
  cv::Size size_grid0(grid_x, grid_y); // width x height
  cv::Mat grid_2d_grid0 = cv::Mat::zeros(size_grid0, CV_8UC1);
  cv::Mat mask0_updated = mask0.clone();
  auto it0 = pts0.begin();
  auto it1 = ids0.begin();
  while (it0 != pts0.end()) {
    // Get current left keypoint, check that it is in bounds
    cv::KeyPoint kpt = *it0;
    int x = (int)kpt.pt.x;
    int y = (int)kpt.pt.y;
    int edge = 10;
    if (x < edge || x >= img0pyr.at(0).cols - edge || y < edge || y >= img0pyr.at(0).rows - edge) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Calculate mask coordinates for close points
    int x_close = (int)(kpt.pt.x / (float)min_px_dist);
    int y_close = (int)(kpt.pt.y / (float)min_px_dist);
    if (x_close < 0 || x_close >= size_close0.width || y_close < 0 || y_close >= size_close0.height) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Calculate what grid cell this feature is in
    int x_grid = std::floor(kpt.pt.x / size_x0);
    int y_grid = std::floor(kpt.pt.y / size_y0);
    if (x_grid < 0 || x_grid >= size_grid0.width || y_grid < 0 || y_grid >= size_grid0.height) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Check if this keypoint is near another point
    if (grid_2d_close0.at<uint8_t>(y_close, x_close) > 127) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Now check if it is in a mask area or not
    // NOTE: mask has max value of 255 (white) if it should be
    if (mask0.at<uint8_t>(y, x) > 127) {
      it0 = pts0.erase(it0);
      it1 = ids0.erase(it1);
      continue;
    }
    // Else we are good, move forward to the next point
    grid_2d_close0.at<uint8_t>(y_close, x_close) = 255;
    if (grid_2d_grid0.at<uint8_t>(y_grid, x_grid) < 255) {
      grid_2d_grid0.at<uint8_t>(y_grid, x_grid) += 1;
    }
    // Append this to the local mask of the image
    if (x - min_px_dist >= 0 && x + min_px_dist < img0pyr.at(0).cols && y - min_px_dist >= 0 && y + min_px_dist < img0pyr.at(0).rows) {
      cv::Point pt1(x - min_px_dist, y - min_px_dist);
      cv::Point pt2(x + min_px_dist, y + min_px_dist);
      cv::rectangle(mask0_updated, pt1, pt2, cv::Scalar(255), -1);
    }
    it0++;
    it1++;
  }

  // First compute how many more features we need to extract from this image
  double min_feat_percent = 0.50;
  int num_featsneeded_0 = num_features - (int)pts0.size();

  // LEFT: if we need features we should extract them in the current frame
  // LEFT: we will also try to track them from this frame over to the right frame
  // LEFT: in the case that we have two features that are the same, then we should merge them
  if (num_featsneeded_0 > std::min(20, (int)(min_feat_percent * num_features))) {

    // This is old extraction code that would extract from the whole image
    // This can be slow as this will recompute extractions for grid areas that we have max features already
    // std::vector<cv::KeyPoint> pts0_ext;
    // Grider_FAST::perform_griding(img0pyr.at(0), mask0_updated, pts0_ext, num_features, grid_x, grid_y, threshold, true);

    // We also check a downsampled mask such that we don't extract in areas where it is all masked!
    cv::Mat mask0_grid;
    cv::resize(mask0, mask0_grid, size_grid0, 0.0, 0.0, cv::INTER_NEAREST);

    // Create grids we need to extract from and then extract our features (use fast with griding)
    int num_features_grid = (int)((double)num_features / (double)(grid_x * grid_y)) + 1;
    int num_features_grid_req = std::max(1, (int)(min_feat_percent * num_features_grid));
    std::vector<std::pair<int, int>> valid_locs;
    for (int x = 0; x < grid_2d_grid0.cols; x++) {
      for (int y = 0; y < grid_2d_grid0.rows; y++) {
        if ((int)grid_2d_grid0.at<uint8_t>(y, x) < num_features_grid_req && (int)mask0_grid.at<uint8_t>(y, x) != 255) {
          valid_locs.emplace_back(x, y);
        }
      }
    }
    std::vector<cv::KeyPoint> pts0_ext;
    Grider_GRID::perform_griding(img0pyr.at(0), mask0_updated, valid_locs, pts0_ext, num_features, grid_x, grid_y, threshold, true);

    // Now, reject features that are close a current feature
    std::vector<cv::KeyPoint> kpts0_new;
    std::vector<cv::Point2f> pts0_new;
    for (auto &kpt : pts0_ext) {
      // Check that it is in bounds
      int x_grid = (int)(kpt.pt.x / (float)min_px_dist);
      int y_grid = (int)(kpt.pt.y / (float)min_px_dist);
      if (x_grid < 0 || x_grid >= size_close0.width || y_grid < 0 || y_grid >= size_close0.height)
        continue;
      // See if there is a point at this location
      if (grid_2d_close0.at<uint8_t>(y_grid, x_grid) > 127)
        continue;
      // Else lets add it!
      grid_2d_close0.at<uint8_t>(y_grid, x_grid) = 255;
      kpts0_new.push_back(kpt);
      pts0_new.push_back(kpt.pt);
    }

    // TODO: Project points from the left frame into the right frame
    // TODO: This will not work for large baseline systems.....
    // TODO: If we had some depth estimates we could do a better projection
    // TODO: Or project and search along the epipolar line??
    std::vector<cv::KeyPoint> kpts1_new;
    std::vector<cv::Point2f> pts1_new;
    kpts1_new = kpts0_new;
    pts1_new = pts0_new;

    // If we have points, do KLT tracking to get the valid projections into the right image
    if (!pts0_new.empty()) {

      // Do our KLT tracking from the left to the right frame of reference
      // NOTE: we have a pretty big window size here since our projection might be bad
      // NOTE: but this might cause failure in cases of repeated textures (eg. checkerboard)
      std::vector<uchar> mask;
      // perform_matching(img0pyr, img1pyr, kpts0_new, kpts1_new, cam_id_left, cam_id_right, mask);
      std::vector<float> error;
      cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
      cv::calcOpticalFlowPyrLK(img0pyr, img1pyr, pts0_new, pts1_new, mask, error, win_size, pyr_levels, term_crit,
                               cv::OPTFLOW_USE_INITIAL_FLOW);

      // Loop through and record only ones that are valid
      for (size_t i = 0; i < pts0_new.size(); i++) {

        // Check to see if the feature is out of bounds (oob) in either image
        bool oob_left = ((int)pts0_new.at(i).x < 0 || (int)pts0_new.at(i).x >= img0pyr.at(0).cols || (int)pts0_new.at(i).y < 0 ||
                         (int)pts0_new.at(i).y >= img0pyr.at(0).rows);
        bool oob_right = ((int)pts1_new.at(i).x < 0 || (int)pts1_new.at(i).x >= img1pyr.at(0).cols || (int)pts1_new.at(i).y < 0 ||
                          (int)pts1_new.at(i).y >= img1pyr.at(0).rows);

        // Check to see if it there is already a feature in the right image at this location
        //  1) If this is not already in the right image, then we should treat it as a stereo
        //  2) Otherwise we will treat this as just a monocular track of the feature
        // TODO: we should check to see if we can combine this new feature and the one in the right
        // TODO: seems if reject features which overlay with right features already we have very poor tracking perf
        if (!oob_left && !oob_right && mask[i] == 1) {
          // update the uv coordinates
          kpts0_new.at(i).pt = pts0_new.at(i);
          kpts1_new.at(i).pt = pts1_new.at(i);
          // append the new uv coordinate
          pts0.push_back(kpts0_new.at(i));
          pts1.push_back(kpts1_new.at(i));
          // move id forward and append this new point
          size_t temp = ++currid;
          ids0.push_back(temp);
          ids1.push_back(temp);
        } else if (!oob_left) {
          // update the uv coordinates
          kpts0_new.at(i).pt = pts0_new.at(i);
          // append the new uv coordinate
          pts0.push_back(kpts0_new.at(i));
          // move id forward and append this new point
          size_t temp = ++currid;
          ids0.push_back(temp);
        }
      }
    }
  }

  // RIGHT: Now summarise the number of tracks in the right image
  // RIGHT: We will try to extract some monocular features if we have the room
  // RIGHT: This will also remove features if there are multiple in the same location
  cv::Size size_close1((int)((float)img1pyr.at(0).cols / (float)min_px_dist), (int)((float)img1pyr.at(0).rows / (float)min_px_dist));
  cv::Mat grid_2d_close1 = cv::Mat::zeros(size_close1, CV_8UC1);
  float size_x1 = (float)img1pyr.at(0).cols / (float)grid_x;
  float size_y1 = (float)img1pyr.at(0).rows / (float)grid_y;
  cv::Size size_grid1(grid_x, grid_y); // width x height
  cv::Mat grid_2d_grid1 = cv::Mat::zeros(size_grid1, CV_8UC1);
  cv::Mat mask1_updated = mask0.clone();
  it0 = pts1.begin();
  it1 = ids1.begin();
  while (it0 != pts1.end()) {
    // Get current left keypoint, check that it is in bounds
    cv::KeyPoint kpt = *it0;
    int x = (int)kpt.pt.x;
    int y = (int)kpt.pt.y;
    int edge = 10;
    if (x < edge || x >= img1pyr.at(0).cols - edge || y < edge || y >= img1pyr.at(0).rows - edge) {
      it0 = pts1.erase(it0);
      it1 = ids1.erase(it1);
      continue;
    }
    // Calculate mask coordinates for close points
    int x_close = (int)(kpt.pt.x / (float)min_px_dist);
    int y_close = (int)(kpt.pt.y / (float)min_px_dist);
    if (x_close < 0 || x_close >= size_close1.width || y_close < 0 || y_close >= size_close1.height) {
      it0 = pts1.erase(it0);
      it1 = ids1.erase(it1);
      continue;
    }
    // Calculate what grid cell this feature is in
    int x_grid = std::floor(kpt.pt.x / size_x1);
    int y_grid = std::floor(kpt.pt.y / size_y1);
    if (x_grid < 0 || x_grid >= size_grid1.width || y_grid < 0 || y_grid >= size_grid1.height) {
      it0 = pts1.erase(it0);
      it1 = ids1.erase(it1);
      continue;
    }
    // Check if this keypoint is near another point
    // NOTE: if it is *not* a stereo point, then we will not delete the feature
    // NOTE: this means we might have a mono and stereo feature near each other, but that is ok
    bool is_stereo = (std::find(ids0.begin(), ids0.end(), *it1) != ids0.end());
    if (grid_2d_close1.at<uint8_t>(y_close, x_close) > 127 && !is_stereo) {
      it0 = pts1.erase(it0);
      it1 = ids1.erase(it1);
      continue;
    }
    // Now check if it is in a mask area or not
    // NOTE: mask has max value of 255 (white) if it should be
    if (mask1.at<uint8_t>(y, x) > 127) {
      it0 = pts1.erase(it0);
      it1 = ids1.erase(it1);
      continue;
    }
    // Else we are good, move forward to the next point
    grid_2d_close1.at<uint8_t>(y_close, x_close) = 255;
    if (grid_2d_grid1.at<uint8_t>(y_grid, x_grid) < 255) {
      grid_2d_grid1.at<uint8_t>(y_grid, x_grid) += 1;
    }
    // Append this to the local mask of the image
    if (x - min_px_dist >= 0 && x + min_px_dist < img1pyr.at(0).cols && y - min_px_dist >= 0 && y + min_px_dist < img1pyr.at(0).rows) {
      cv::Point pt1(x - min_px_dist, y - min_px_dist);
      cv::Point pt2(x + min_px_dist, y + min_px_dist);
      cv::rectangle(mask1_updated, pt1, pt2, cv::Scalar(255), -1);
    }
    it0++;
    it1++;
  }

  // RIGHT: if we need features we should extract them in the current frame
  // RIGHT: note that we don't track them to the left as we already did left->right tracking above
  int num_featsneeded_1 = num_features - (int)pts1.size();
  if (num_featsneeded_1 > std::min(20, (int)(min_feat_percent * num_features))) {

    // This is old extraction code that would extract from the whole image
    // This can be slow as this will recompute extractions for grid areas that we have max features already
    // std::vector<cv::KeyPoint> pts1_ext;
    // Grider_FAST::perform_griding(img1pyr.at(0), mask1_updated, pts1_ext, num_features, grid_x, grid_y, threshold, true);

    // We also check a downsampled mask such that we don't extract in areas where it is all masked!
    cv::Mat mask1_grid;
    cv::resize(mask1, mask1_grid, size_grid1, 0.0, 0.0, cv::INTER_NEAREST);

    // Create grids we need to extract from and then extract our features (use fast with griding)
    int num_features_grid = (int)((double)num_features / (double)(grid_x * grid_y)) + 1;
    int num_features_grid_req = std::max(1, (int)(min_feat_percent * num_features_grid));
    std::vector<std::pair<int, int>> valid_locs;
    for (int x = 0; x < grid_2d_grid1.cols; x++) {
      for (int y = 0; y < grid_2d_grid1.rows; y++) {
        if ((int)grid_2d_grid1.at<uint8_t>(y, x) < num_features_grid_req && (int)mask1_grid.at<uint8_t>(y, x) != 255) {
          valid_locs.emplace_back(x, y);
        }
      }
    }
    std::vector<cv::KeyPoint> pts1_ext;
    Grider_GRID::perform_griding(img1pyr.at(0), mask1_updated, valid_locs, pts1_ext, num_features, grid_x, grid_y, threshold, true);

    // Now, reject features that are close a current feature
    for (auto &kpt : pts1_ext) {
      // Check that it is in bounds
      int x_grid = (int)(kpt.pt.x / (float)min_px_dist);
      int y_grid = (int)(kpt.pt.y / (float)min_px_dist);
      if (x_grid < 0 || x_grid >= size_close1.width || y_grid < 0 || y_grid >= size_close1.height)
        continue;
      // See if there is a point at this location
      if (grid_2d_close1.at<uint8_t>(y_grid, x_grid) > 127)
        continue;
      // Else lets add it!
      pts1.push_back(kpt);
      size_t temp = ++currid;
      ids1.push_back(temp);
      grid_2d_close1.at<uint8_t>(y_grid, x_grid) = 255;
    }
  }
}

void TrackKLT::perform_matching(const std::vector<cv::Mat> &img0pyr, const std::vector<cv::Mat> &img1pyr, std::vector<cv::KeyPoint> &kpts0,
                                std::vector<cv::KeyPoint> &kpts1, size_t id0, size_t id1, std::vector<uchar> &mask_out) {

  // We must have equal vectors
  assert(kpts0.size() == kpts1.size());

  // Return if we don't have any points
  if (kpts0.empty() || kpts1.empty())
    return;

  // Convert keypoints into points (stupid opencv stuff)
  std::vector<cv::Point2f> pts0, pts1;
  for (size_t i = 0; i < kpts0.size(); i++) {
    pts0.push_back(kpts0.at(i).pt);
    pts1.push_back(kpts1.at(i).pt);
  }

  // If we don't have enough points for ransac just return empty
  // We set the mask to be all zeros since all points failed RANSAC
  if (pts0.size() < 10) {
    for (size_t i = 0; i < pts0.size(); i++)
      mask_out.push_back((uchar)0);
    return;
  }

  // Now do KLT tracking to get the valid new points
  std::vector<uchar> mask_klt;
  std::vector<float> error;
  cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
  cv::calcOpticalFlowPyrLK(img0pyr, img1pyr, pts0, pts1, mask_klt, error, win_size, pyr_levels, term_crit, cv::OPTFLOW_USE_INITIAL_FLOW);

  // Normalize these points, so we can then do ransac
  // We don't want to do ransac on distorted image uvs since the mapping is nonlinear
  std::vector<cv::Point2f> pts0_n, pts1_n;
  for (size_t i = 0; i < pts0.size(); i++) {
    pts0_n.push_back(camera_calib.at(id0)->undistort_cv(pts0.at(i)));
    pts1_n.push_back(camera_calib.at(id1)->undistort_cv(pts1.at(i)));
  }

  // Do RANSAC outlier rejection (note since we normalized the max pixel error is now in the normalized cords)
  std::vector<uchar> mask_rsc;
  double max_focallength_img0 = std::max(camera_calib.at(id0)->get_K()(0, 0), camera_calib.at(id0)->get_K()(1, 1));
  double max_focallength_img1 = std::max(camera_calib.at(id1)->get_K()(0, 0), camera_calib.at(id1)->get_K()(1, 1));
  double max_focallength = std::max(max_focallength_img0, max_focallength_img1);
  cv::findFundamentalMat(pts0_n, pts1_n, cv::FM_RANSAC, 2.0 / max_focallength, 0.999, mask_rsc);

  // Loop through and record only ones that are valid
  for (size_t i = 0; i < mask_klt.size(); i++) {
    auto mask = (uchar)((i < mask_klt.size() && mask_klt[i] && i < mask_rsc.size() && mask_rsc[i]) ? 1 : 0);
    mask_out.push_back(mask);
  }

  // Copy back the updated positions
  for (size_t i = 0; i < pts0.size(); i++) {
    kpts0.at(i).pt = pts0.at(i);
    kpts1.at(i).pt = pts1.at(i);
  }
}
