#include "parameters.h"

#include <cmath>
#include <iostream>

#include <opencv2/opencv.hpp>

int FT_ROW = 400;
int FT_COL = 800;
int FT_FOCAL_LENGTH = 460;

std::string FISHEYE_MASK;
int MAX_CNT = 150;
int MIN_DIST = 30;
int FT_WINDOW_SIZE = 10;
int FREQ = 10;
double F_THRESHOLD = 1.0;
int SHOW_TRACK = 0;
int STEREO_TRACK = 0;
int EQUALIZE = 1;
int FISHEYE = 0;
bool PUB_THIS_FRAME = true;

void readFeatureParameters(const std::string& config_path)
{
    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "feature tracker: cannot open " << config_path << std::endl;
        return;
    }

    fs["image_width"] >> FT_COL;
    fs["image_height"] >> FT_ROW;
    fs["max_cnt"] >> MAX_CNT;
    fs["min_dist"] >> MIN_DIST;
    fs["freq"] >> FREQ;
    fs["F_threshold"] >> F_THRESHOLD;
    fs["show_track"] >> SHOW_TRACK;
    fs["equalize"] >> EQUALIZE;
    fs["fisheye"] >> FISHEYE;
    fs["fisheye_mask"] >> FISHEYE_MASK;

    cv::FileNode pp = fs["projection_parameters"];
    if (!pp.empty())
    {
        double fx = 460.0;
        pp["fx"] >> fx;
        FT_FOCAL_LENGTH = static_cast<int>(std::lround(fx));
    }

    STEREO_TRACK = 0;
    PUB_THIS_FRAME = true;
    fs.release();
}
