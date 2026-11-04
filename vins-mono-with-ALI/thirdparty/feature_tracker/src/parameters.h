#pragma once

#include <string>

extern int FT_ROW;
extern int FT_COL;
extern int FT_FOCAL_LENGTH;

extern std::string FISHEYE_MASK;
extern int MAX_CNT;
extern int MIN_DIST;
extern int FT_WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

// ROS-free replacement for the feature_tracker node's readParameters(NodeHandle&).
void readFeatureParameters(const std::string& config_path);
