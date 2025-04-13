// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/light_corner_corrector.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{
class Detector
{
public:
  struct EnhanceParams
  {
    // 是否开启增强
    bool is_enhance;
    bool is_using_pca;
    // 增强参数
    // 数字的期望灰度值
    double target_gray_number_value;
    // 灯条的期望灰度值
    double target_gray_light_value;
  };

  struct LightParams
  {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
    double min_fill_ratio;
  };

  struct ArmorParams
  {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a, const EnhanceParams & enhance_params);

  std::vector<Armor> detect(const cv::Mat & input);

  cv::Mat preprocessImage(const cv::Mat & input);
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  void reextractArmors(std::vector<Armor> & armors);
  Light extractLight_Max_fromBinary_minAreaRect(cv::Mat & binary_img);
  Light extractLight_Max_fromBinary_Least_Square(cv::Mat & binary_img);

  // For debug usage
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat & img);

  int binary_thres;
  int detect_color;
  LightParams l;
  ArmorParams a;
  EnhanceParams enhance_params;
  LightCornerCorrector light_corner_corrector;

  std::unique_ptr<NumberClassifier> classifier;

  // Debug msgs
  cv::Mat binary_img;
  cv::Mat gray_img;
  auto_aim_interfaces::msg::DebugLights debug_lights;
  auto_aim_interfaces::msg::DebugArmors debug_armors;

private:
  bool isLight(const Light & possible_light);
  bool containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
  ArmorType isArmor(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
