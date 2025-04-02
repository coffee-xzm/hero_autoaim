#ifndef ARMOR_DETECTOR__REFINE_YAW_HPP_
#define ARMOR_DETECTOR__REFINE_YAW_HPP_

#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Core"

using namespace std;
class RefineYaw
{
public:
    RefineYaw(float pitch_gimbal, float pitch, float yaw_init, float & yaw_res,
                cv::Mat & tvec, vector<cv::Point2f> raw_points, vector<cv::Point3f> object_points,
                const cv::Mat& cameraMatrix, const cv::Mat& distortionCoeffs);
    cv::Mat CVMatrix(Eigen::MatrixXd matrix);
    Eigen::MatrixXd MatrixCV(cv::Mat cv_mat);
private:
    float pitch_, pitch_gimbal_;
    float yaw_left_, yaw_right_, yaw_mid1_, yaw_mid2_;
    float yaw_, deviation_last_;
    float deviation_left_, deviation_right_, deviation_mid1_, deviation_mid2_;
    int max_iterations_;
    float max_truncation_error_;
    float delta_, epsilon_, mu_;
    cv::Mat tvec_;
    vector<cv::Point2f> raw_points_;
    vector<cv::Point3f> object_points_;
    cv::Mat cameraMatrix_;
    cv::Mat distortionCoeffs_;

    float update(void);
    bool single_step(void);
    float get_deviation(float yaw);
    Eigen::MatrixXd eulerToRotationMatrixOpticalRad(double yaw_, double pitch_, double roll_);
};
#endif  // ARMOR_DETECTOR__REFINE_YAW_HPP_
