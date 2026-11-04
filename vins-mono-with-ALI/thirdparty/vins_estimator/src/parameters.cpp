#include "parameters.h"

#include <iostream>

double FOCAL_LENGTH = 460.0;
double INIT_DEPTH = 5.0;
double MIN_PARALLAX = 10.0 / 460.0;
int ESTIMATE_EXTRINSIC = 0;

double ACC_N = 0.08, ACC_W = 0.00004;
double GYR_N = 0.004, GYR_W = 2.0e-6;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
Eigen::Vector3d G(0.0, 0.0, 9.81);

double BIAS_ACC_THRESHOLD = 0.1;
double BIAS_GYR_THRESHOLD = 0.1;
double SOLVER_TIME = 0.04;
int NUM_ITERATIONS = 8;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double TD = 0.0;
double TR = 0.0;
int ESTIMATE_TD = 0;
int ROLLING_SHUTTER = 0;
double ROW = 400.0;
double COL = 800.0;

void readParameters(const std::string& config_path)
{
    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "estimator: cannot open " << config_path << std::endl;
        return;
    }

    fs["max_solver_time"] >> SOLVER_TIME;
    fs["max_num_iterations"] >> NUM_ITERATIONS;
    fs["keyframe_parallax"] >> MIN_PARALLAX;
    fs["acc_n"] >> ACC_N;
    fs["acc_w"] >> ACC_W;
    fs["gyr_n"] >> GYR_N;
    fs["gyr_w"] >> GYR_W;
    fs["g_norm"] >> G.z();
    fs["image_height"] >> ROW;
    fs["image_width"] >> COL;
    fs["estimate_extrinsic"] >> ESTIMATE_EXTRINSIC;
    fs["td"] >> TD;
    fs["estimate_td"] >> ESTIMATE_TD;
    fs["rolling_shutter"] >> ROLLING_SHUTTER;
    fs["rolling_shutter_tr"] >> TR;

    double fx = 460.0;
    cv::FileNode pp = fs["projection_parameters"];
    if (!pp.empty())
        pp["fx"] >> fx;
    FOCAL_LENGTH = fx;
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    RIC.clear();
    TIC.clear();
    if (ESTIMATE_EXTRINSIC == 2)
    {
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
    }
    else
    {
        cv::Mat cv_R, cv_T;
        fs["extrinsicRotation"] >> cv_R;
        fs["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        cv::cv2eigen(cv_R, R);
        cv::cv2eigen(cv_T, T);
        Eigen::Quaterniond Q(R);
        R = Q.normalized().toRotationMatrix();
        RIC.push_back(R);
        TIC.push_back(T);
    }

    // The node created an output file in readParameters.  The port's driver owns the trajectory path,
    // so these stay empty and no file is created here.
    EX_CALIB_RESULT_PATH.clear();
    VINS_RESULT_PATH.clear();
    IMU_TOPIC.clear();
    fs.release();
}
