/**
 * @file    detector_node.h
 * @brief   装甲板识别节点
 *
 * @author  Zhang-Haopeng
 * @date    2024-4-10
 * @version v1.0
 */

#ifndef __DETECTOR_NODE_H
#define __DETECTOR_NODE_H

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
/* ROS2库 */
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
/* OpenCV库 */
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

/* msg头文件 */
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "auto_aim_interfaces/msg/armors.hpp"
//#include "sentry_interfaces/msg/decision.hpp"
//#include "auto_aim_interfaces/msg/set_mode.hpp"

/* STD库 */
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

/* User库 */
#include "armor_detector/armor.hpp"
#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"

namespace rm_auto_aim
{
enum class EnemyColor {
  RED = 0,
  BLUE = 1,
};

enum VisionMode {
  AUTO_AIM_RED = 0,
  AUTO_AIM_BLUE = 1,
  RUNE_RED = 2,
  RUNE_BLUE = 3,
  INIT = 4,
};

/* 类定义 --------------------------------------------------------------------------------------------------------------*/
/**
 * @brief  装甲板识别节点类
 */
class Class_Armor_Detector : public rclcpp::Node
{
public:
    Class_Armor_Detector(const rclcpp::NodeOptions & options);

private:
    //void decisionCallback(const sentry_interfaces::msg::Decision::SharedPtr msg) ;
    // rclcpp::Subscription<sentry_interfaces::msg::Decision>::SharedPtr decision_sub_;

    rcl_interfaces::msg::SetParametersResult Parameters_Callback(const std::vector<rclcpp::Parameter> & parameters);

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    std::unique_ptr<Detector> initDetector();
    std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

    void createDebugPublishers();
    void destroyDebugPublishers();

    void publishMarkers();

    float orientationToPitchDirect(const geometry_msgs::msg::Quaternion & q);

    //void setModeCallback(const auto_aim_interfaces::msg::SetMode::ConstSharedPtr response);

    // Armor Detector
    std::unique_ptr<Detector> detector_;

    // Dynamic color
    EnemyColor detect_color_ ;
    VisionMode last_vision_mode_ = VisionMode::INIT;
    int is_auto_aim_;
    // Enable/Disable Rune Detector
    //rclcpp::Subscription<auto_aim_interfaces::msg::SetMode>::SharedPtr set_auto_aim_mode_sub_;

    // Detected armors publisher
    auto_aim_interfaces::msg::Armors armors_msg_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

    // Visualization marker publisher
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Camera info part
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    cv::Point2f cam_center_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
    std::unique_ptr<PnPSolver> pnp_solver_;

    // Image subscrpition
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    // Debug information
    bool debug_;
    std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
    image_transport::Publisher binary_img_pub_;
    image_transport::Publisher number_img_pub_;
    image_transport::Publisher result_img_pub_;

    // Tf
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    OnSetParametersCallbackHandle::SharedPtr        /*!< 参数触发事件句柄 */
    params_callback_handle_;
};

inline std::string visionModeToString(VisionMode mode) {
  switch (mode) {
    case VisionMode::AUTO_AIM_RED:
      return "AUTO_AIM_RED";
    case VisionMode::AUTO_AIM_BLUE:
      return "AUTO_AIM_BLUE";
    case VisionMode::RUNE_RED:
      return "RUNE_RED";
    case VisionMode::RUNE_BLUE:
      return "RUNE_BLUE";
    default:
      return "UNKNOWN";
  }
}
}  // namespace rm_auto_aim

#endif	/* detector_node.h */
