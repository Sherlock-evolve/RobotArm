#pragma once

#include <string>
#include <vector>
#include <fstream>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_arm_hw
{

/**
 * @brief ServoSystem - ros2_control 硬件接口实现类
 * 
 * 功能：将 ROS2 控制栈的关节角度命令（弧度）转换为舵机脉宽（纳秒），并写入 /dev/servo_pulse 设备
 * 
 * 工作流程：
 * 1. on_init(): 初始化，加载硬件参数（设备路径、关节配置等）
 * 2. export_state_interfaces(): 导出状态接口（供控制器读取当前关节位置）
 * 3. export_command_interfaces(): 导出命令接口（供控制器写入目标关节位置）
 * 4. on_activate(): 激活硬件，打开设备文件
 * 5. read(): 读取当前状态（目前是伪反馈：直接复制命令值）
 * 6. write(): 将命令角度转换为脉宽，写入硬件设备
 * 7. on_deactivate(): 停用硬件，关闭设备文件
 */
class ServoSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ServoSystem)

  /**
   * @brief 初始化硬件接口
   * @param info 硬件信息（包含关节列表、硬件参数等）
   * @return 成功返回 SUCCESS，失败返回 ERROR
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief 导出状态接口（供控制器读取关节位置）
   * @return 状态接口列表，每个关节一个 position 接口
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief 导出命令接口（供控制器写入目标关节位置）
   * @return 命令接口列表，每个关节一个 position 接口
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief 激活硬件（打开设备文件）
   * @param previous_state 之前的状态
   * @return 成功返回 SUCCESS，失败返回 ERROR
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 停用硬件（关闭设备文件）
   * @param previous_state 之前的状态
   * @return 成功返回 SUCCESS
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 读取硬件状态（目前是伪反馈：直接复制命令值）
   * @param time 当前时间
   * @param period 控制周期
   * @return 成功返回 OK
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief 写入命令到硬件（角度转脉宽，写入设备）
   * @param time 当前时间
   * @param period 控制周期
   * @return 成功返回 OK，失败返回 ERROR
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /**
   * @brief 关节配置结构体
   * 存储每个关节的标定参数，用于角度到脉宽的映射
   */
  struct JointConfig
  {
    std::string name;          // 关节名称（如 "shoulder_pan_joint"）
    int servo_id{0};            // 舵机 ID（对应 /dev/servo_pulse 的通道号）
    double angle_min{0.0};      // 关节角度最小值（弧度）
    double angle_max{0.0};      // 关节角度最大值（弧度）
    double pulse_min{0.0};      // 对应 angle_min 的脉宽（纳秒）
    double pulse_max{0.0};      // 对应 angle_max 的脉宽（纳秒）
    double pulse_zero{0.0};     // 角度为 0 时的脉宽（纳秒，锚点）
    int direction{1};           // 方向：1 表示正方向，-1 表示反向
    double offset{0.0};         // 角度偏移量（弧度），用于机械零点校准
  };

  std::string device_path_;              // 设备文件路径（如 "/dev/servo_pulse"）
  bool dry_run_{false};                  // 干运行模式：true 时不打开设备，只打印日志（用于 PC 测试）
  bool dry_run_print_{false};            // 干运行打印开关：true 时在 dry_run 模式下打印调试信息
  std::vector<JointConfig> joint_cfgs_;  // 所有关节的配置列表
  std::vector<double> cmd_position_;     // 命令位置数组（弧度），由控制器写入
  std::vector<double> state_position_;   // 状态位置数组（弧度），供控制器读取（目前是伪反馈）
  std::ofstream dev_stream_;              // 设备文件输出流
  
  // 节流机制：确保写入频率与舵机频率匹配（50 Hz = 每 20 ms 一次）
  rclcpp::Time last_write_time_;          // 上次写入设备的时间
  static constexpr int64_t SERVO_UPDATE_PERIOD_NS = 20000000LL;  // 50 Hz = 20 ms = 20000000 纳秒

  /**
   * @brief 从硬件参数中加载配置
   * 读取 device、dry_run、joints 等参数，解析 YAML 格式的关节配置
   * @return 成功返回 SUCCESS，失败返回 ERROR
   */
  hardware_interface::CallbackReturn load_configs();

  /**
   * @brief 打开设备文件
   * @return 成功返回 SUCCESS，失败返回 ERROR（dry_run 模式下直接返回 SUCCESS）
   */
  hardware_interface::CallbackReturn open_device();

  /**
   * @brief 关闭设备文件
   */
  void close_device();

  /**
   * @brief 将值限制在 [lo, hi] 范围内
   * @param v 输入值
   * @param lo 最小值
   * @param hi 最大值
   * @return 限制后的值
   */
  static double clamp(double v, double lo, double hi);
};

}  // namespace my_arm_hw

