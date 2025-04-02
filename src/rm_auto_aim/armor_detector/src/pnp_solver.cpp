// Copyright 2022 Chen Jun

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_auto_aim
{
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

/************************************************************************************************************************
 * @brief   PnP解算函数
 ***********************************************************************************************************************/
bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec, float pitch_gimbal)
{
    std::vector<cv::Point2f> image_armor_points;

    // Fill in image points
    image_armor_points.emplace_back(armor.left_light.bottom);
    image_armor_points.emplace_back(armor.left_light.top);
    image_armor_points.emplace_back(armor.right_light.top);
    image_armor_points.emplace_back(armor.right_light.bottom);

    // Solve pnp
    auto object_points = (armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_);
    // calculate time cost of pnp
    bool success = cv::solvePnP(object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
    if (!success)
    {
        return false;
    }
    // Get roll, pitch, yaw
    cv::Mat r;
    cv::Rodrigues(rvec, r);
    double roll, pitch, yaw;
    RotationMatrixToGroundEuler(r, yaw, pitch, roll, pitch_gimbal);
    // cout << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << endl;
    // Recalculate rvec
    // float yaw_threshold = 10.0 / 180.0 * 3.14;
    // if (-yaw_threshold < yaw && yaw < yaw_threshold){
    float dist = pow(tvec.at<double>(0, 0), 2) + pow(tvec.at<double>(1, 0), 2) + pow(tvec.at<double>(2, 0), 0.5);
    if(dist>1.2){
      // pitch_gimbal: negative when uprised
      // pitch(armor): positive when uprised
      float pitch_real;
      if (armor.number == "outpose"){
          pitch_real = -CV_PI / 12;
      }
      else{
          pitch_real = CV_PI / 12;
      }
      // Refine yaw
      float yaw_res;
      // calculate time cost of refine_yaw
      RefineYaw refine_yaw((float)pitch_gimbal, pitch_real, (float)yaw, yaw_res, 
                          tvec, image_armor_points, object_points, 
                          camera_matrix_, dist_coeffs_);
      // std::cout << "raw yaw: " << yaw << ", refined yaw: " << yaw_res << std::endl;
      cv::Mat refined_r = refine_yaw.CVMatrix(GroundEulerToRotationMatrixRad(pitch_gimbal, yaw_res, pitch_real, 0));
      cv::Rodrigues(refined_r, rvec);
    }
    
    return true;
}

/************************************************************************************************************************
 * @brief   计算到中心距离函数
 ***********************************************************************************************************************/
float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}

Eigen::MatrixXd PnPSolver::GroundEulerToRotationMatrixRad(double pitch_gimbal_, double yaw_, double pitch_, double roll_) {
  
  Eigen::MatrixXd rotation_matrix(3, 3),
                  rotation_x(3, 3),
                  rotation_y(3, 3),
                  rotation_z(3, 3),
                  rotation_right_yaw(3, 3),
                  rotation_right_roll(3, 3),
                  rotation_pitch_gimbal(3, 3);

  double cY = cos(yaw_);
  double sY = sin(yaw_);
  double cP = cos(pitch_);
  double sP = sin(pitch_);
  double cR = cos(roll_);
  double sR = sin(roll_);

  rotation_x << 1.0, 0.0, 0.0,
                0.0, cR, -sR,  
                0.0, sR, cR;

  rotation_y << cP, 0.0, sP,
                0.0, 1.0, 0.0,
                -sP, 0.0, cP;

  rotation_z << cY, -sY, 0.0,
                sY, cY, 0.0,
                0.0, 0.0, 1.0;
    
  rotation_right_roll << 1, 0, 0,
                         0, 0, -1,
                         0, 1, 0;

  rotation_right_yaw << 0, -1, 0,
                        1, 0, 0,
                        0, 0, 1;
  
  rotation_pitch_gimbal << cos(pitch_gimbal_), 0, -sin(pitch_gimbal_),
                            0, 1, 0,
                            sin(pitch_gimbal_), 0, cos(pitch_gimbal_);

  rotation_matrix = rotation_right_roll * rotation_right_yaw * rotation_pitch_gimbal * rotation_z * rotation_y * rotation_x;

  return rotation_matrix;
}

cv::Mat PnPSolver::CVMatrix(Eigen::MatrixXd matrix) {
  cv::Mat cv_matrix(matrix.rows(), matrix.cols(), CV_64FC1);
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      cv_matrix.at<double>(i, j) = matrix(i, j);
    }
  }
  return cv_matrix;
}

void PnPSolver::RotationMatrixToEuler(const cv::Mat & r, double & yaw, double & pitch, double & roll) {
  tf2::Matrix3x3 tf_r = tf2::Matrix3x3(
    r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2), r.at<double>(1, 0), r.at<double>(1, 1),
    r.at<double>(1, 2), r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2));
    // roll pi/2, yaw pi/2
    tf2::Matrix3x3 rotation_roll = tf2::Matrix3x3(
        1, 0, 0,
        0, 0, 1,
        0, -1, 0);
    tf2::Matrix3x3 rotation_yaw = tf2::Matrix3x3(
        0, 1, 0,
        -1, 0, 0,
        0, 0, 1);
    tf2::Matrix3x3 rotation_r = rotation_yaw * rotation_roll * tf_r;
    // double roll, pitch, yaw;
    rotation_r.getRPY(roll, pitch, yaw);
}

void PnPSolver::RotationMatrixToGroundEuler(const cv::Mat & r, double & yaw, double & pitch, double & roll, float pitch_gimbal) {
  tf2::Matrix3x3 tf_r = tf2::Matrix3x3(
    r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2), r.at<double>(1, 0), r.at<double>(1, 1),
    r.at<double>(1, 2), r.at<double>(2, 0), r.at<double>(2, 1), r.at<double>(2, 2));
    // roll pi/2, yaw pi/2
    tf2::Matrix3x3 rotation_roll = tf2::Matrix3x3(
        1, 0, 0,
        0, 0, 1,
        0, -1, 0);
    tf2::Matrix3x3 rotation_yaw = tf2::Matrix3x3(
        0, 1, 0,
        -1, 0, 0,
        0, 0, 1);
    tf2::Matrix3x3 rotation_pitch = tf2::Matrix3x3(
        cos(pitch_gimbal), 0, sin(pitch_gimbal),
        0, 1, 0,
        -sin(pitch_gimbal), 0, cos(pitch_gimbal));
    tf2::Matrix3x3 rotation_r = rotation_pitch * rotation_yaw * rotation_roll * tf_r;
    // double roll, pitch, yaw;
    rotation_r.getRPY(roll, pitch, yaw);
}

}  // namespace rm_auto_aim
