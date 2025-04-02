/**
 * @file    detector_node.cpp
 * @brief   装甲板识别节点
 *
 * @author  Zhang-Haopeng
 * @date    2024-4-10
 * @version v1.0
 */

/* 头文件引用 ----------------------------------------------------------------------------------------------------------*/
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
/************************************************************************************************************************
 * @brief   装甲板识别节点类构造函数
 ***********************************************************************************************************************/
// Class_Armor_Detector::Class_Armor_Detector(const rclcpp::NodeOptions & options) : Node("armor_detector", options),decision_sub_(this->create_subscription<sentry_interfaces::msg::Decision>(
//       "/sentry_decision/Decision", 10, std::bind(&Class_Armor_Detector::decisionCallback, this, std::placeholders::_1))) , is_auto_aim_(true)
Class_Armor_Detector::Class_Armor_Detector(const rclcpp::NodeOptions & options) : Node("armor_detector", options) , is_auto_aim_(true)
{
    RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

    /* 初始化识别器 */
    detector_ = initDetector();

    /* 创建发布者、订阅者对象 */
    armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>("/detector/armors", rclcpp::SensorDataQoS());

    // tf
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    armor_marker_.ns = "armors";
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.05;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.g = 0.5;
    armor_marker_.color.b = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.01);

    text_marker_.ns = "classification";
    text_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z = 0.1;
    text_marker_.color.a = 1.0;
    text_marker_.color.r = 1.0;
    text_marker_.color.g = 1.0;
    text_marker_.color.b = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.01);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

    /* Debug使能参数声明 */
    debug_ = this->declare_parameter("debug", false);
    if (debug_)
    {
        createDebugPublishers();
    }

    /* 动态检测Debug使能参数 */
    debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
        debug_ = p.as_bool();
        debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

    // 动态参数回调函数绑定
    this->params_callback_handle_ = 
        this->add_on_set_parameters_callback(std::bind(&Class_Armor_Detector::Parameters_Callback, this, std::placeholders::_1));

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
        cam_info_sub_.reset();
    });

    // set_auto_aim_mode_sub_ = this->create_subscription<auto_aim_interfaces::msg::SetMode>(
    //     "set_mode", rclcpp::SensorDataQoS(),
    //     std::bind(&Class_Armor_Detector::setModeCallback, this, std::placeholders::_1));

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&Class_Armor_Detector::imageCallback, this, std::placeholders::_1));
}



// void Class_Armor_Detector::decisionCallback(const sentry_interfaces::msg::Decision::SharedPtr msg) {

//     if(detector_ == nullptr || detector_->classifier == nullptr)
//         return;
//     detector_->classifier->ignore_classes_ = std::vector<std::string>{"negative"};
//     if(!msg->hit_enermy_engineer)
//         detector_->classifier->ignore_classes_.push_back(std::string("2"));
//     if(!msg->hit_enermy_sentry)
//         detector_->classifier->ignore_classes_.push_back(std::string("guard"));
//     if(!msg->hit_enermy_base)
//         detector_->classifier->ignore_classes_.push_back(std::string("base"));
//     if(!msg->hit_enermy_outpost)
//         detector_->classifier->ignore_classes_.push_back(std::string("outpost"));
// }


/************************************************************************************************************************
 * @brief   原始图像消息订阅回调函数
 ***********************************************************************************************************************/
void Class_Armor_Detector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    // std::cout << "Armor imageCallback" << std::endl;

    if(is_auto_aim_ == false){
        return;
    }

    auto_aim_interfaces::msg::Armor armor_msg;

    /* 检测一帧中全部装甲板 */
    auto armors = detectArmors(img_msg);

    if (pnp_solver_ == nullptr)
    {
        return;
    }

    /* 消息初始化 */
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;

    /* 对全部装甲板进行pnp解算 */
    for (const auto & armor : armors)
    {
        cv::Mat rvec, tvec;

        /* 进行pnp解算 */
        // TODO: calculate real pitch angle of the gimbal
        float pitch_gimbal;
        try
        {
            rclcpp::Time time = img_msg->header.stamp;
            auto transform = tf2_buffer_->lookupTransform("odom", "camera_link", time, rclcpp::Duration::from_seconds(0.1));
            pitch_gimbal = orientationToPitchDirect(transform.transform.rotation);
            // std::cout << "self yaw: " << self_position(3) << std::endl;
        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
            return;
        }
        bool pnp_result = pnp_solver_->solvePnP(armor, rvec, tvec, pitch_gimbal);
        if (pnp_result)
        {
            // Fill basic info
            armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
            armor_msg.number = armor.number;

            // Fill pose
            armor_msg.pose.position.x = tvec.at<double>(0);
            armor_msg.pose.position.y = tvec.at<double>(1);
            armor_msg.pose.position.z = tvec.at<double>(2);

            // 保存装甲板在相机坐标系下的y坐标，用于后续的一辆车之中的装甲板标记
            armor_msg.y_coordinate_in_camera_frame = -1.0 * armor_msg.pose.position.x;
            
            // rvec to 3x3 rotation matrix
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);
            // rotation matrix to quaternion
            tf2::Matrix3x3 tf2_rotation_matrix(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                rotation_matrix.at<double>(2, 2));
            tf2::Quaternion tf2_q;
            tf2_rotation_matrix.getRotation(tf2_q);
            armor_msg.pose.orientation = tf2::toMsg(tf2_q);
            armor_msg.base_link_pose = armor_msg.pose;

            // Fill the distance to image center
            armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

            // Fill the markers
            armor_marker_.id++;
            armor_marker_.scale.y = (armor.type == ArmorType::SMALL ? 0.135 : 0.23);
            armor_marker_.pose = armor_msg.pose;
            text_marker_.id++;
            text_marker_.pose.position = armor_msg.pose.position;
            text_marker_.pose.position.y -= 0.1;
            text_marker_.text = armor.classfication_result;
            armors_msg_.armors.emplace_back(armor_msg);
            marker_array_.markers.emplace_back(armor_marker_);
            marker_array_.markers.emplace_back(text_marker_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "PnP failed!");
        }
    }

    // Publishing detected armors
    armors_pub_->publish(armors_msg_);

    // Publishing marker
    publishMarkers();
}

/************************************************************************************************************************
 * @brief   识别器初始化函数
 ***********************************************************************************************************************/
std::unique_ptr<Detector> Class_Armor_Detector::initDetector()
{
    rcl_interfaces::msg::ParameterDescriptor param_desc;

    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;
    int binary_thres = declare_parameter("binary_thres", 160, param_desc);

    param_desc.description = "0-RED, 1-BLUE";
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 1;
    auto detect_color = declare_parameter("detect_color", RED, param_desc);
    detect_color_ = static_cast<EnemyColor>(detect_color);

    Detector::LightParams l_params = {
        .min_ratio = declare_parameter("light.min_ratio", 0.1),
        .max_ratio = declare_parameter("light.max_ratio", 0.4),
        .max_angle = declare_parameter("light.max_angle", 40.0),
        .min_fill_ratio = declare_parameter("light.min_fill_ratio", 0.785)};

    Detector::ArmorParams a_params = {
        .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
        .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
        .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
        .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
        .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
        .max_angle = declare_parameter("armor.max_angle", 35.0)};

    Detector::EnhanceParams enhance_params = {
        .is_enhance = this->declare_parameter("is_using_enhancement", false),
        .is_using_pca = this->declare_parameter("is_using_pca", false),
        .target_gray_number_value = declare_parameter("target_gray_number_value", 10.0),
        .target_gray_light_value = declare_parameter("target_gray_light_value", 160.0)};

    auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params, enhance_params);

    /* 初始化数字分类模型 */
    auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
    auto model_path = pkg_path + "/model/lenet.onnx";
    auto label_path = pkg_path + "/model/label.txt";
    double threshold = this->declare_parameter("classifier_threshold", 0.7);
    std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
    detector->classifier = std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

    return detector;
}

/************************************************************************************************************************
 * @brief   装甲板检测函数
 ***********************************************************************************************************************/
std::vector<Armor> Class_Armor_Detector::detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
    /* 转换成CV */
    auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

    /* 参数获取 */
    detector_->binary_thres = get_parameter("binary_thres").as_int();
    // detector_->detect_color = get_parameter("detect_color").as_int();
    detector_->detect_color = (int)detect_color_;
    detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

    /* 检测装甲板 */
    auto armors = detector_->detect(img);

    /* 计算延迟（摄像头获取-检测装甲板完成） */
    auto final_time = this->now();
    auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

    /* 调试信息处理并发布 */
    if (debug_)
    {
        /* 发布二值化图像数据 */
        binary_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

        /* 灯条装甲板信息按x坐标排序并发布 */
        std::sort(
            detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
            [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
        std::sort(
            detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
            [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });
        lights_data_pub_->publish(detector_->debug_lights);
        armors_data_pub_->publish(detector_->debug_armors);

        /* 发布处理得到的装甲板数字图片 */
        if (!armors.empty())
        {
            auto all_num_img = detector_->getAllNumbersImage();
            number_img_pub_.publish(*cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
        }

        /* 绘制识别结果 */
        detector_->drawResults(img);

        /* 绘制相机中心 */
        cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);

        /* 绘制延迟 */
        std::stringstream latency_ss;
        latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
        auto latency_s = latency_ss.str();
        cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
    }

    return armors;
}

/************************************************************************************************************************
 * @brief   创建Debug发布者函数
 ***********************************************************************************************************************/
void Class_Armor_Detector::createDebugPublishers()
{
    lights_data_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
    armors_data_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

    binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
    number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
    result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

/************************************************************************************************************************
 * @brief   销毁Debug发布者函数
 ***********************************************************************************************************************/
void Class_Armor_Detector::destroyDebugPublishers()
{
    lights_data_pub_.reset();
    armors_data_pub_.reset();

    binary_img_pub_.shutdown();
    number_img_pub_.shutdown();
    result_img_pub_.shutdown();
}

/************************************************************************************************************************
 * @brief   Makers消息发布函数
 ***********************************************************************************************************************/
void Class_Armor_Detector::publishMarkers()
{
    // using Marker = visualization_msgs::msg::Marker;

    // armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
    // marker_array_.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array_);
}

/************************************************************************************************************************
 * @brief   欧拉角转Pitch角
 ***********************************************************************************************************************/
float Class_Armor_Detector::orientationToPitchDirect(const geometry_msgs::msg::Quaternion & q){
    tf2::Quaternion tf2_q;
    tf2::fromMsg(q, tf2_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    return pitch;

}


/************************************************************************************************************************
 * @brief   模式设置回调函数
 ***********************************************************************************************************************/
// void Class_Armor_Detector::setModeCallback(const auto_aim_interfaces::msg::SetMode::ConstSharedPtr response)
// {
//     VisionMode mode = static_cast<VisionMode>(response->mode);

//     if(mode == last_vision_mode_){
//         return;
//     }

//     std::string mode_name = visionModeToString(mode);

//     // std::cout << "armor mode: " << mode_name << std::endl;

//     if (mode_name == "UNKNOWN") {
//         // FYT_ERROR("rune_detector", "Invalid mode: {}", request->mode);
//         RCLCPP_ERROR(get_logger(), "Invalid mode: {%d}", response->mode);
//         return;
//     }

//     // auto createImageSub = [this]() {
//     //     if (img_sub_ == nullptr) {
//     //     img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//     //         "image_raw",
//     //         rclcpp::SensorDataQoS(),
//     //         std::bind(&Class_Armor_Detector::imageCallback, this, std::placeholders::_1));
//     //     }
//     // };

//     if(mode == VisionMode::AUTO_AIM_BLUE && last_vision_mode_ == VisionMode::AUTO_AIM_RED){
//         detect_color_ = EnemyColor::BLUE;
//     }
//     else if(mode == VisionMode::AUTO_AIM_RED && last_vision_mode_ == VisionMode::AUTO_AIM_BLUE){
//         detect_color_ = EnemyColor::RED;
//     }
//     else{
//         switch (mode) {
//             case VisionMode::AUTO_AIM_RED: {
//                 detect_color_ = EnemyColor::RED;
//                 is_auto_aim_ = true;
//                 // createImageSub();
//                 break;
//             }
//             case VisionMode::AUTO_AIM_BLUE: {
//                 detect_color_ = EnemyColor::BLUE;
//                 is_auto_aim_ = true;
//                 // createImageSub();
//                 break;
//             }
//             default: {
//                 // img_sub_.reset();
//                 is_auto_aim_ = false;
//                 // img_sub_.reset();
//                 break;
//             }
//         }
//     }

//     last_vision_mode_ = mode;
// }





/************************************************************************************************************************
 * @brief   参数回调函数
 ***********************************************************************************************************************/
rcl_interfaces::msg::SetParametersResult Class_Armor_Detector::Parameters_Callback(const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters)
	{
		if (param.get_name() == "target_gray_number_value")
		{
            this->detector_->enhance_params.target_gray_number_value = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Set target_gray_number_value successfully! target_gray_number_value = %.4f", param.as_double());
		}
		else if (param.get_name() == "target_gray_light_value")
		{
            this->detector_->enhance_params.target_gray_light_value = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Set target_gray_light_value successfully! target_gray_light_value = %.4f", param.as_double());
		}
		else
		{
			result.successful = false;
			result.reason = "Unknown parameter: " + param.get_name();
		}
    }
    return result;
}




}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::Class_Armor_Detector)
