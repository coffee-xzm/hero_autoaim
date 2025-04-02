#include "armor_detector/refine_yaw.hpp"
#include "opencv2/opencv.hpp"
#include "iostream"

using namespace std;
RefineYaw::RefineYaw(float pitch_gimbal, float pitch, float yaw_init, float & yaw_res,
                cv::Mat & tvec, vector<cv::Point2f> raw_points, vector<cv::Point3f> object_points,
                const cv::Mat& cameraMatrix, const cv::Mat& distortionCoeffs):
pitch_(pitch),pitch_gimbal_(pitch_gimbal), tvec_(tvec), raw_points_(raw_points), object_points_(object_points),
cameraMatrix_(cameraMatrix), distortionCoeffs_(distortionCoeffs) 
{
    // Initialize the parameters
    max_iterations_ = 20;
    max_truncation_error_ = 1e-1;
    delta_ = 1e-6;
    epsilon_ = 1e-4;
    mu_ = 1000000;
    // Initialize the variables
    // float radius;
    // if (yaw_init < -20.0 / 180.0 * 3.14 || yaw_init > 20.0 / 180.0 * 3.14){
    //     radius = 2.0 / 180.0 * 3.14;
    // }
    // else{
    //     radius = 2.0 / 180.0 * 3.14;
    // }
    // yaw_left_ = yaw_init - radius;
    // yaw_right_ = yaw_init + radius;
    // yaw_left_ = - 3.14 / 2.0;
    // yaw_right_ = 3.14 / 2.0;
    // deviation_left_ = get_deviation(yaw_left_);
    // deviation_right_ = get_deviation(yaw_right_);
    // Start the iteration
    // yaw_res = yaw_init;
    yaw_ = yaw_init;
    deviation_last_ = get_deviation(yaw_);
    // cout << "pitch_gimbal: " << pitch_gimbal_ << endl;
    // cout << "deviation init: " << deviation_last_ << endl;
    yaw_res = update();
}

float RefineYaw::update(void) {
    float iterations_ = 0;
    while (iterations_ < max_iterations_) {
        if (single_step()) {
            // cout<<"exit at step: "<<iterations_<<endl;
            // cout << "deviation final: "<<get_deviation(yaw_) << endl;
            // cout<<"refined yaw: "<<(yaw_mid1_ + yaw_mid2_) / 2<<endl;
            // cout << deviation_mid1_ << " " << deviation_mid2_ << endl;
            // return (yaw_mid1_ + yaw_mid2_) / 2;
            return yaw_;
        }
        iterations_++;
    }
            // cout<<"exit at step: "<<iterations_<<endl;
            // cout << "deviation final: "<<get_deviation(yaw_) << endl;
    return yaw_;
    // return (yaw_mid1_ + yaw_mid2_) / 2;
}

bool RefineYaw::single_step(void) {
    // yaw_mid1_ = yaw_left_ + 1.0 * (yaw_right_ - yaw_left_) / 3.0;
    // yaw_mid2_ = yaw_left_ + 2.0 * (yaw_right_ - yaw_left_) / 3.0;
    // deviation_mid1_ = get_deviation(yaw_mid1_);
    // deviation_mid2_ = get_deviation(yaw_mid2_);
    // if (deviation_mid1_ < deviation_mid2_) {
    //     // if (deviation_mid1_ < deviation_left_) {
    //     //     // Rising order of deviation: mid1 < (mid2, left)
    //     //     yaw_right_ = yaw_mid2_;
    //     //     deviation_right_ = deviation_mid2_;
    //     // } else {
    //     //     // Rising order of deviation: left < mid1 < mid2
    //     //     yaw_right_ = yaw_mid1_;
    //     //     deviation_right_ = deviation_mid1_;
    //     //     yaw_left_ = yaw_left_ - (yaw_right_ - yaw_left_) / 2.0;
    //     //     deviation_left_ = get_deviation(yaw_left_);
    //     // }
    //     yaw_right_ = yaw_mid2_;
    //     deviation_right_ = deviation_mid2_;
    // } else {
    //     // if (deviation_mid2_ < deviation_right_) {
    //     //     // Rising order of deviation: mid2 < (mid1, right)
    //     //     yaw_left_ = yaw_mid1_;
    //     //     deviation_left_ = deviation_mid1_;
    //     // } else {
    //     //     // Rising order of deviation: right < mid2 < mid1
    //     //     yaw_left_ = yaw_mid2_;
    //     //     deviation_left_ = deviation_mid2_;
    //     //     yaw_right_ = yaw_right_ + (yaw_right_ - yaw_left_) / 2.0;
    //     //     deviation_right_ = get_deviation(yaw_right_);
    //     // }
    //     yaw_left_ = yaw_mid1_;
    //     deviation_left_ = deviation_mid1_;
    // }
    // if (yaw_right_ - yaw_left_ < max_truncation_error_){
    //     // success, no need to continue
    //     return true;
    // }
    float d_1_ = get_deviation(yaw_ + delta_);
    float d_2_ = get_deviation(yaw_ - delta_);
    float grad_ = (d_1_ - d_2_) / (2 * delta_);
    if(fabs(grad_) < max_truncation_error_){
        return true;
    }
    deviation_last_ = (d_1_ + d_2_) / 2.0;
    yaw_ -= deviation_last_ * grad_ / (grad_ * grad_ + mu_);
    // yaw_ -= grad_ * epsilon_;
    // continue
    return false;
}

float RefineYaw::get_deviation(float yaw){
    float roll = 0;
    float pitch = pitch_;
    double deviation = 0.0; // 要使它最小
    Eigen::MatrixXd rotation = eulerToRotationMatrixOpticalRad(yaw, pitch, roll);
    vector<cv::Point2f> projected_points;
    cv::Mat distortion_(1, 5, CV_64FC1);
    distortion_.at<double>(0, 0) = distortion_.at<double>(0, 1) = distortion_.at<double>(0, 2) = distortion_.at<double>(0, 3) = distortion_.at<double>(0, 4) = 0;
    cv::projectPoints(object_points_, CVMatrix(rotation), tvec_, cameraMatrix_, distortion_, projected_points);
    // cv::projectPoints(object_points_, CVMatrix(rotation), tvec_, cameraMatrix_, distortionCoeffs_, projected_points);
    // cout << "tvec: " << tvec_ << CVMatrix(eulerToRotationMatrixOpticalRad(0, 0, 0).transpose() * MatrixCV(tvec_)) << endl;
    for(int i = 0; i < 4; i++) {
        // cout << "projected_points: " << projected_points[i] << ", raw points: " << raw_points_[i] << endl;
        deviation += std::pow(projected_points[i].x - raw_points_[i].x, 2) + std::pow(projected_points[i].y - raw_points_[i].y, 2); // 转换到像素坐标系进行计算
        // deviation += std::pow(projected_points[i].x - raw_points_[i].x, 2); // 转换到像素坐标系进行计算
    }
    return deviation;
}

cv::Mat RefineYaw::CVMatrix(Eigen::MatrixXd matrix) {
  cv::Mat cv_matrix(matrix.rows(), matrix.cols(), CV_64FC1);
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      cv_matrix.at<double>(i, j) = matrix(i, j);
    }
  }
  return cv_matrix;
}

Eigen::MatrixXd RefineYaw::MatrixCV(cv::Mat cv_mat) {
    Eigen::MatrixXd matrix_(cv_mat.rows, cv_mat.cols);
    for (int i = 0; i < cv_mat.rows; i++) {
        for (int j = 0; j < cv_mat.cols; j++) {
        matrix_(i, j) = cv_mat.at<double>(i, j);
        }
    }
    return matrix_;
}

Eigen::MatrixXd RefineYaw::eulerToRotationMatrixOpticalRad(double yaw_, double pitch_, double roll_) {
  
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

//   rotation_matrix = rotation_z * rotation_y * rotation_x * rotation_pitch_gimbal * rotation_right_yaw * rotation_right_roll;
  rotation_matrix = rotation_right_roll * rotation_right_yaw * rotation_pitch_gimbal * rotation_z * rotation_y * rotation_x ;

  return rotation_matrix;
}
