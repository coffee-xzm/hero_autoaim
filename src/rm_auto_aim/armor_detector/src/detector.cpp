// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/detector.hpp"
#include "auto_aim_interfaces/msg/debug_armor.hpp"
#include "auto_aim_interfaces/msg/debug_light.hpp"

namespace rm_auto_aim
{
Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a, const EnhanceParams & enhance_params)
: binary_thres(bin_thres), detect_color(color), l(l), a(a), enhance_params(enhance_params)
{
}

std::vector<Armor> Detector::detect(const cv::Mat & input)
{
  binary_img = preprocessImage(input);
  lights_ = findLights(input, binary_img);
  armors_ = matchLights(lights_);

  // std::cout << "start to classify" << std::endl;
  if (!armors_.empty()){
    classifier->extractNumbers(input, armors_);
    classifier->classify(armors_);
  }

  // std::cout << "start to enhance" << std::endl;
  // 进行灯条重提取
  if(this->enhance_params.is_enhance){
    if( !armors_.empty()){
      reextractArmors(armors_);
    }
    // To seperate the armor that is_error
    armors_.erase(std::remove_if(armors_.begin(), armors_.end(), [](const Armor & armor) { return armor.is_error; }), armors_.end());
  }

  // std::cout << "start to draw" << std::endl;
  // 进行灯条角点重提取
  if(this->enhance_params.is_using_pca){
    if( !armors_.empty()){
      for(auto & armor : armors_){
        light_corner_corrector.correctCorners(armor, gray_img);
      }
    }
  }

  // std::cout << std::endl;
  return armors_;
}

cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img)
{
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}

std::vector<Light> Detector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (isLight(light)) {
      auto rect = light.boundingRect();
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rbg_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rbg_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE;
        // 计算contour内的平均灰度值
        int count = 0;
        for (auto point : contour)
        {
            light.avg_gray_in_contour += gray_img.at<uchar>(point);
            count++;
        }
        light.avg_gray_in_contour /= count;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

bool Detector::isLight(const Light & light)
{
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok && light.width > 10;

  // Fill in debug information
  auto_aim_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;
  this->debug_armors.data.clear();

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color) continue;

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  // Fill in debug information
  auto_aim_interfaces::msg::DebugArmor armor_data;
  armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  return type;
}

/************************************************************************************************************************
 * @brief   利用装甲板的数字信息增强装甲板并且重新提取灯条
 ***********************************************************************************************************************/
void Detector::reextractArmors(std::vector<Armor> & armors)
{
    for(auto & armor : armors)
    {
        // 计算roi二值化后白色区域在原来的灰度图中的平均灰度值
        // mean_gray_value = np.mean(gray_image_roi[binary_image_roi == 255])
        double avg_gray_value_number = cv::mean(armor.number_img_gray, armor.number_img)[0];
        double target_gray_value = this->enhance_params.target_gray_number_value;
        double target_gray_light_value = this->enhance_params.target_gray_light_value;
        // gamma2 = math.log2(60/255)/math.log2(mean_gray_value/255)
        double gamma_number=1, gamma_light_diff=1, gamma_light_standard =1;

        if(target_gray_value > 0){
            gamma_number = std::log2(target_gray_value * 1.0f / 255) / std::log2(avg_gray_value_number * 1.0f / 255);
        } 
        // 根据两个灯条中灰度大的那个计算需要调整的gamma值，log2(大)/log2(小)
        gamma_light_diff = std::log2(std::max(armor.left_light.avg_gray_in_contour, armor.right_light.avg_gray_in_contour) * 1.0f / 255) /
                            std::log2(std::min(armor.left_light.avg_gray_in_contour, armor.right_light.avg_gray_in_contour) * 1.0f / 255);
        if(target_gray_light_value > 0){
            gamma_light_standard = std::log2(target_gray_light_value * 1.0f / 255) / 
                            std::log2(std::max(armor.left_light.avg_gray_in_contour, armor.right_light.avg_gray_in_contour) * 1.0f / 255);

        }
        // std::cout << "gamma light: " << gamma_light_diff << std::endl;
        // std::cout << "larger gray value light: " <<  (armor.left_light.avg_gray_in_contour > armor.right_light.avg_gray_in_contour ? armor.left_light.avg_gray_in_contour : armor.right_light.avg_gray_in_contour) << std::endl;
        // double gamma_light_diff = 1.0f;
        // double classifier_thresh_adjuest = math.pow(100/255,1/gamma2)*255
        double gamma_left = gamma_number * gamma_light_standard * ( armor.left_light.avg_gray_in_contour > armor.right_light.avg_gray_in_contour ? 1 : gamma_light_diff );
        double gamma_right = gamma_number * gamma_light_standard * ( armor.left_light.avg_gray_in_contour < armor.right_light.avg_gray_in_contour ? 1 : gamma_light_diff );

        // double classifier_thresh_adjuest = std::pow(binary_thres * 1.0f / 255, 1.0f / gamma_number) * 255;
        double classifier_thresh_adjuest_left = std::pow(binary_thres * 1.0f / 255, 1.0f / gamma_left) * 255;
        double classifier_thresh_adjuest_right = std::pow(binary_thres * 1.0f / 255, 1.0f / gamma_right) * 255;


        // 获取比灯条原来的外接矩形更大的外接矩形，用于再次提取灯条，用左上和右下顶点表示
        // alpha_width = 1.5
        // alpha_height = 1.2
        // point_lu = (int(min(top_middle[0], bottom_middle[0]) - alpha_width * width/2), int((top_middle[1] + bottom_middle[1])/2 - alpha_height * length/2))
        // point_rb = (int(max(top_middle[0], bottom_middle[0]) + alpha_width * width/2), int((top_middle[1] + bottom_middle[1])/2 + alpha_height * length/2))
        double alpha_width = 1.5;
        double alpha_height = 1.2;
        // 先根据左边灯条提取外接矩形
        cv::Point2d left_point_lu = cv::Point2d(
            std::min(armor.left_light.top.x, armor.left_light.bottom.x) - alpha_width * armor.left_light.width / 2,
            (armor.left_light.top.y + armor.left_light.bottom.y) / 2 - alpha_height * armor.left_light.length / 2);
        cv::Point2d left_point_rb = cv::Point2d(
            std::max(armor.left_light.top.x, armor.left_light.bottom.x) + alpha_width * armor.left_light.width / 2,
            (armor.left_light.top.y + armor.left_light.bottom.y) / 2 + alpha_height * armor.left_light.length / 2);
        // 限制外接矩形在原图(gray_img)内 
        left_point_lu.x = std::max(0.0, left_point_lu.x);
        left_point_lu.y = std::max(0.0, left_point_lu.y);
        left_point_rb.x = std::min(gray_img.cols - 1.0, left_point_rb.x);
        left_point_rb.y = std::min(gray_img.rows - 1.0, left_point_rb.y);
        // 再根据右边灯条提取外接矩形
        cv::Point2d right_point_lu = cv::Point2d(
            std::min(armor.right_light.top.x, armor.right_light.bottom.x) - alpha_width * armor.right_light.width / 2,
            (armor.right_light.top.y + armor.right_light.bottom.y) / 2 - alpha_height * armor.right_light.length / 2);
        cv::Point2d right_point_rb = cv::Point2d(
            std::max(armor.right_light.top.x, armor.right_light.bottom.x) + alpha_width * armor.right_light.width / 2,
            (armor.right_light.top.y + armor.right_light.bottom.y) / 2 + alpha_height * armor.right_light.length / 2);
        // 限制外接矩形在原图(gray_img)内
        right_point_lu.x = std::max(0.0, right_point_lu.x);
        right_point_lu.y = std::max(0.0, right_point_lu.y);
        right_point_rb.x = std::min(gray_img.cols - 1.0, right_point_rb.x);
        right_point_rb.y = std::min(gray_img.rows - 1.0, right_point_rb.y);
        
        // 根据原本的图像提取出左右灯条的roi
        cv::Mat left_roi = gray_img(cv::Rect(left_point_lu, left_point_rb)).clone();
        cv::Mat right_roi = gray_img(cv::Rect(right_point_lu, right_point_rb)).clone();
        // 二值化
        cv::threshold(left_roi, left_roi, classifier_thresh_adjuest_left, 255, cv::THRESH_BINARY);
        cv::threshold(right_roi, right_roi, classifier_thresh_adjuest_right, 255, cv::THRESH_BINARY);

        // 重新提取灯条
        auto new_left_light = extractLight_Max_fromBinary_minAreaRect(left_roi);
        auto new_right_light = extractLight_Max_fromBinary_minAreaRect(right_roi);

        if(new_left_light.is_error || new_right_light.is_error)
        {
            armor.is_error = 1;
            continue;
        }

        // 更新装甲板的灯条信息，需要考虑到roi的偏移
        // 看灯条是否大幅度变化，定义大幅度为距离变化大于alpha_height
        if(new_left_light.length == 0 || new_right_light.length == 0)
        {
            continue;
        }
        double height_change_rate_left = (new_left_light.length) / armor.left_light.length;
        height_change_rate_left = height_change_rate_left > 1 ? height_change_rate_left : 1 / height_change_rate_left;
        double height_change_rate_right = (new_right_light.length) / armor.right_light.length;
        height_change_rate_right = height_change_rate_right > 1 ? height_change_rate_right : 1 / height_change_rate_right;

        if(height_change_rate_left <= alpha_height)
        {   
            armor.left_light.top.x = new_left_light.top.x + left_point_lu.x;
            armor.left_light.top.y = new_left_light.top.y + left_point_lu.y;
            armor.left_light.bottom.x = new_left_light.bottom.x + left_point_lu.x;
            armor.left_light.bottom.y = new_left_light.bottom.y + left_point_lu.y;
            armor.left_light.length = new_left_light.length;
            armor.left_light.width = new_left_light.width;
            armor.left_light.tilt_angle = new_left_light.tilt_angle;
            armor.left_light.enhancing_gamma = gamma_left;
        }
        
        if(height_change_rate_right <= alpha_height)
        {
            armor.right_light.top.x = new_right_light.top.x + right_point_lu.x;
            armor.right_light.top.y = new_right_light.top.y + right_point_lu.y;
            armor.right_light.bottom.x = new_right_light.bottom.x + right_point_lu.x;
            armor.right_light.bottom.y = new_right_light.bottom.y + right_point_lu.y;
            armor.right_light.length = new_right_light.length;
            armor.right_light.width = new_right_light.width;
            armor.right_light.tilt_angle = new_right_light.tilt_angle;
            armor.right_light.enhancing_gamma = gamma_right;
        }

        // 更新装甲板的中心点
        armor.center = (armor.left_light.center + armor.right_light.center) / 2;
        
    }
}   

Light Detector::extractLight_Max_fromBinary_minAreaRect(cv::Mat & binary_img)
{
    // 根据find light流程提取局部图片中最大的灯条
    using std::vector;
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if(contours.empty())
    {
        auto light = Light(cv::RotatedRect());
        light.is_error = 1;
        return light;
    }

    // 将countours根据面积排序，第一个为最大的元素
    std::sort(contours.begin(), contours.end(), [](const vector<cv::Point> & a, const vector<cv::Point> & b) {
        return cv::contourArea(a) > cv::contourArea(b);
    });
    // 取最大的contour
    auto r_rect = cv::minAreaRect(contours[0]);
    auto light = Light(r_rect);

    return light;
}

Light Detector::extractLight_Max_fromBinary_Least_Square(cv::Mat & binary_img)
{
    // 根据find light流程提取局部图片中最大的灯条
    using std::vector;
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 将countours根据面积排序，第一个为最大的元素
    std::sort(contours.begin(), contours.end(), [](const vector<cv::Point> & a, const vector<cv::Point> & b) {
        return cv::contourArea(a) > cv::contourArea(b);
    });
    // 取最大的contour
    auto r_rect = cv::minAreaRect(contours[0]);

    // 利用contour的轮廓拟合一条直线(fitLine)
    cv::Point2f top, bottom;
    cv::Vec4f line_para;
    cv::fitLine(contours[0], line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
    // 计算直线的斜率，确保不为无穷也不能为0
    if ( std::abs(line_para[1]) > 1e-2 && std::abs(line_para[0]) > 1e-2 )
    {
        // top_y 为contour[0]的y最小值，bottom_y 为contour[0]的y最大值
        top.y = std::numeric_limits<int>::max();
        bottom.y = std::numeric_limits<int>::min();
        for (cv::Point& pt : contours[0]) {
            top.y = std::min(top.y, pt.y * 1.0f);
            bottom.y = std::max(bottom.y, pt.y * 1.0f);
        }
        double k = line_para[1] / line_para[0];
        double b = line_para[3] - k * line_para[2];
        top.x = (top.y - b) / k;
        bottom.x = (bottom.y - b) / k;

        return Light(r_rect, top, bottom);
    }
    else
    {
        return Light(r_rect);
    }
    

    auto light = Light(r_rect);

    return light;
}

cv::Mat Detector::getAllNumbersImage()
{
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

void Detector::drawResults(cv::Mat & img)
{
  // Draw Lights
  for (const auto & light : lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  for (const auto & armor : armors_) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  for (const auto & armor : armors_) {
    cv::putText(
      img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace rm_auto_aim
