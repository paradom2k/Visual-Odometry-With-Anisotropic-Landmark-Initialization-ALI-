#include "ali.h"

#include <algorithm>
#include <cmath>

namespace ali {

Frame::Frame(const cv::Mat& gray, const Config& config, double focal_length)
    : config_(config), focal_length_(focal_length) {
  if (config_.window < 3) config_.window = 3;
  if (config_.window % 2 == 0) config_.window += 1;
  if (!gray.empty()) {
    cv::Mat gray_f;
    if (gray.type() != CV_32F) {
      gray.convertTo(gray_f, CV_32F);
    } else {
      gray_f = gray;
    }
    cv::Sobel(gray_f, gx_, CV_32F, 1, 0, 3);
    cv::Sobel(gray_f, gy_, CV_32F, 0, 1, 3);
  }
}

bool Frame::ComputeTensor(const cv::Point2f& pt, Eigen::Matrix2d* M) const {
  const int half = config_.window / 2;
  const int cx = static_cast<int>(std::lround(pt.x));
  const int cy = static_cast<int>(std::lround(pt.y));
  if (gx_.empty() || cx - half < 0 || cy - half < 0 ||
      cx + half >= gx_.cols || cy + half >= gx_.rows) {
    return false;
  }

  double sxx = 0.0, sxy = 0.0, syy = 0.0;
  for (int y = cy - half; y <= cy + half; ++y) {
    const float* px = gx_.ptr<float>(y);
    const float* py = gy_.ptr<float>(y);
    for (int x = cx - half; x <= cx + half; ++x) {
      const double ix = px[x];
      const double iy = py[x];
      sxx += ix * ix;
      sxy += ix * iy;
      syy += iy * iy;
    }
  }
  (*M)(0, 0) = sxx;
  (*M)(0, 1) = sxy;
  (*M)(1, 0) = sxy;
  (*M)(1, 1) = syy;
  return std::isfinite(sxx) && std::isfinite(sxy) && std::isfinite(syy);
}

bool Frame::whiten(const cv::Point2f& pt, Eigen::Matrix2d* sqrt_info) {
  if (!config_.enabled) {
    *sqrt_info = (focal_length_ / config_.sigma_pix) * Eigen::Matrix2d::Identity();
    return true;
  }

  Eigen::Matrix2d M;
  if (!ComputeTensor(pt, &M)) return false;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(M);
  if (eig.info() != Eigen::Success) return false;
  const double lambda2 = eig.eigenvalues()(0);
  const double lambda1 = eig.eigenvalues()(1);
  if (!(lambda2 > 1e-9) || !(lambda1 > 0.0)) return false;

  if (!reference_initialized_) {
    reference_l2_ = lambda2;
    reference_initialized_ = true;
  } else {
    reference_l2_ = (1.0 - config_.ref_alpha) * reference_l2_ + config_.ref_alpha * lambda2;
  }

  const double c1 = std::min(config_.clip_hi, std::max(config_.clip_lo, reference_l2_ / lambda1));
  const double c2 = std::min(config_.clip_hi, std::max(config_.clip_lo, reference_l2_ / lambda2));
  const Eigen::Vector2d inv_sqrt(1.0 / std::sqrt(c1), 1.0 / std::sqrt(c2));
  const Eigen::Matrix2d U = eig.eigenvectors();
  *sqrt_info = (focal_length_ / config_.sigma_pix) * U * inv_sqrt.asDiagonal() * U.transpose();
  return true;
}

}  // namespace ali
