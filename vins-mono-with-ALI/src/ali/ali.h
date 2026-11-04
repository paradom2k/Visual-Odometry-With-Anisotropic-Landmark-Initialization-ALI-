#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace ali {

struct Config {
  bool enabled = true;
  int window = 15;          // odd; the structure tensor is summed over window x window
  double sigma_pix = 1.5;   // same nominal pixel sigma as VINS-Mono's sqrt_info baseline
  double clip_lo = 0.25;
  double clip_hi = 4.0;
  double ref_alpha = 0.005;
};

// Computes the ALI whitening matrix for one frame.  The frame owns the Sobel derivatives, so the
// per-feature cost is a window sum and a 2x2 eigendecomposition.  The scene reference is a member because
// it is updated once per new feature, exactly as in the paper's recursion.
class Frame {
 public:
  Frame(const cv::Mat& gray, const Config& config, double focal_length);

  // Returns R_i^{-1/2} premultiplied by focal_length/sigma_pix (the factor ProjectionFactor expects for a
  // normalized-coordinate residual).  Returns false when the patch is out of bounds or degenerate.
  bool whiten(const cv::Point2f& pt, Eigen::Matrix2d* sqrt_info);

  bool enabled() const { return config_.enabled; }
  double reference_l2() const { return reference_l2_; }

 private:
  bool ComputeTensor(const cv::Point2f& pt, Eigen::Matrix2d* M) const;

  Config config_;
  double focal_length_;
  cv::Mat gx_;
  cv::Mat gy_;
  double reference_l2_ = 0.0;
  bool reference_initialized_ = false;
};

}  // namespace ali
