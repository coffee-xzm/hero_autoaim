/**
 * @file    light_corner_corrector.hpp
 * @brief   交点重新提取节点
 *
 * @author  ZhangHaopeng
 * @date    2024/6/20
 * @version v1.0
 */


#ifndef LIGHT_CORNER_CORRECTOR_HPP_
#define LIGHT_CORNER_CORRECTOR_HPP_

#include "opencv2/opencv.hpp"

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{


struct SymmetryAxis {
  cv::Point2f centroid;
  cv::Point2f direction;
  float mean_val; // Mean brightness
};

class LightCornerCorrector {
public:
  explicit LightCornerCorrector() noexcept {}

  // Correct the corners of the armor's lights
  void correctCorners(Armor &armor, const cv::Mat &gray_img);

private:
  // Correct the direction of axis
  void Refind_Axis_Direction(SymmetryAxis &axis);

  // Find the symmetry axis of the light
  SymmetryAxis findSymmetryAxis(const cv::Mat &gray_img, const Light &light);

  // Find the corner of the light
  cv::Point2f findCorner(const cv::Mat &gray_img,
                         const Light &light,
                         const SymmetryAxis &axis,
                         std::string order);
};


}


#endif