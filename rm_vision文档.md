## 前置讲解
为了方便我们阅读源码，可以先来了解下ros2工程的架构配置：
#### 1.节点及其接口
```
// eg,推荐的节点设计模式
class ArmorDetectorNode : public rclcpp::Node
{
public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions & options)
  : Node("armor_detector", options)
  {
    // 接口，参数初始化
    this->declare_parameters();
    this->init_subscriptions();
    this->init_publishers();
  }

private:
  // 各种方法定义
};
```
#### 2.参数管理
```
void declare_parameters()
{
  // 声明并获取参数
  this->declare_parameter("detect_color", 0);
  this->declare_parameter("binary_thres", 160);
  
  // 使用参数描述添加更多信息
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "Debug mode";
  this->declare_parameter("debug", false, param_desc);
  
  // 设置参数回调
  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ArmorDetectorNode::parameters_callback, this, std::placeholders::_1));
    //这里bind将函数和类绑定，类接收到信息时执行回调函数，其他部分我们也能看到lamda表达式对回调函数的实现
}
```
这里声明的常量其实是作为备用的，参数可以统一管理在.yaml文件中
#### 3.组件化设计
比如刚才提到的yaml参数管理，还有文件末尾：
```
// 在源文件末尾注册组件
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
```
或者是预设配置，如：SensorDataQoS，表明对于通信行为:1.可靠性:尽力而为 2.历史策略:保留少量最新信息 3.持久?:不存储数据。

# 项目思路
## rm_serial_driver
### packet.hpp
```
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1; //重置跟踪
  uint8_t reserved : 6; //似乎没有用到，考虑为其他命令预留
  float roll;
  float pitch;
  float yaw;
  float aim_x; //下位机发布的击打点位置
  float aim_y;
  float aim_z;
  uint16_t checksum = 0; //crc校验相关
} __attribute__((packed));
```
```
struct SendPacket
{
  uint8_t header = 0xA5;
  bool tracking : 1; //跟踪状态
  uint8_t id : 3;          // 0-outpost前哨站 1-5机器人编号 6-guard烧饼 7-base基地
  uint8_t armors_num : 3;  // 2-balance平衡步兵 3-outpost前哨站 4-normal普通机器人
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1; //小陀螺转动时通过卡尔曼滤波计算，不动时为定值
  float r2;
  float dz; //可能是重力补偿或高低装甲板，不同车型的补偿
  uint16_t checksum = 0;
} __attribute__((packed));
```
#### 语法细节
__attribute__((packed));表示不进行内存对齐，优化性能
fromVector和toVector实现了结构体与字节流之间的双向转化
reinterpret_cast表示直接将其二进制位解释为新类型，不进行实质地转化，
而static_cast进行类型检查，保持数据转换语义上的正确，在底层通过汇编指令等转换，
dynamic_cast访问虚函数表，遍历继承层次图从而计算出正确的指针或返回null,const_cast仅移除const修饰符
### rm_serial_driver.cpp
#### 类RMSerialDriver
主要维护packet.hpp，设置了两个线程从而同时读取和写入串口，
同时发布解算于ReceivePacket中roll pitch yaw的从odom到gimbal_link的坐标tf变换与时间戳与可视化中的击打点。
之后hpp中的函数都如字面意思
#### 语法细节
detector_param_client_这一个异步参数客户端可以在不阻塞主线程的情况下修改另一个节点的参数
异常处理：try{检测代码}catch(异常类型)throw 抛出错误。既使错误暂时不阻碍主线程，也能做相关处理便于debug
## rm_gimbal_description
其中的.xacro文件描述了本工程会用到的坐标变换
## rm_auto_aim
又想了下感觉代码不用讲多少，主要有个思路，之后跟着官方文档等就可以开写了。
他的思路是通过传统视觉，灰度，二值提取灯条，然后根据比例进行形态学变换取出中间的数字用以识别。识别时采用了预训练好的机器学习模型，参数在onnx文件中这样进行装甲板与机器类型识别。
这样就发布一个识别的信息。另一方面，通过pnp解算直接推出整车的方位，同时使用卡尔曼滤波进行整车的跟踪。这样又是一个发布的信息。
这两个信息与下位机定时发来的位姿结合，从而解算出所有的坐标。至于为什么识别和解算要分开，一是分开时能保留不同识别的时间戳，更多时间上的信息防止误判，如向左跟踪时相机画面中相对的向右移动，又向右急转，二是更容易对齐下位机发来的时间戳。