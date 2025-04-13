// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>

namespace rm_auto_aim
{
const int RED = 0;
const int BLUE = 1;

enum class ArmorType { SMALL, LARGE, INVALID };
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

struct Light : public cv::RotatedRect
{
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;
        enhancing_gamma = 1.0f;
        is_error = 0;
    }

    // 重载满足根据最小二乘法分割灯条，获取准确的上下中心点信息
    explicit Light(cv::RotatedRect box,cv::Point2f top, cv::Point2f bottom) 
    : cv::RotatedRect(box), top(top), bottom(bottom)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });

        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);

        axis = top - bottom;
        axis = axis / cv::norm(axis);

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;
        enhancing_gamma = 1.0f;
    }

    int color;
    cv::Point2f top, bottom;
    cv::Point2f axis;
    double length;
    double width;
    float tilt_angle;
    double avg_gray_in_contour;
    double enhancing_gamma;
    bool is_error;
};

struct Armor
{
    Armor() = default;
    Armor(const Light & l1, const Light & l2)
    {
        if (l1.center.x < l2.center.x)
        {
            left_light = l1, right_light = l2;
        }
        else
        {
            left_light = l2, right_light = l1;
        }
        center = (left_light.center + right_light.center) / 2;
        is_error = 0;
    }

    // Light pairs part
    Light left_light, right_light;
    cv::Point2f center;
    ArmorType type;

    // Number part
    cv::Mat number_img;
    cv::Mat number_img_gray;
    std::string number;
    float confidence;
    std::string classfication_result;
    bool is_error;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
