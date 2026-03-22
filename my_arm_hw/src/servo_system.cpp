#include "my_arm_hw/servo_system.hpp"

#include <algorithm>
#include <utility>
#include <sstream>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "yaml-cpp/yaml.h"

namespace my_arm_hw
{

namespace
{
const char * kLoggerName = "ServoSystem";
}

/**
 * @brief 将值限制在 [lo, hi] 范围内
 * @param v 输入值
 * @param lo 最小值
 * @param hi 最大值
 * @return 限制后的值（如果 v < lo 返回 lo，如果 v > hi 返回 hi，否则返回 v）
 */
double ServoSystem::clamp(double v, double lo, double hi)
{
  return std::min(std::max(v, lo), hi);
}

/**
 * @brief 从硬件参数中加载配置
 * 
 * 读取以下参数：
 * - device: 设备文件路径（如 "/dev/servo_pulse"）
 * - dry_run: 是否启用干运行模式（可选，默认 false）
 * - dry_run_print: 干运行模式下是否打印调试信息（可选，默认 false）
 * - joints: YAML 格式的关节配置列表
 * 
 * 关节配置必须包含：
 * - name: 关节名称（必须与 URDF 中的关节名一致）
 * - servo_id: 舵机 ID
 * - angle_min/angle_max: 角度范围（弧度）
 * - pulse_min_ns/pulse_max_ns: 脉宽范围（纳秒）
 * - pulse_zero_ns: 零角度对应的脉宽（纳秒）
 * - direction: 方向（1 或 -1）
 * - offset_rad: 角度偏移（弧度）
 * 
 * @return 成功返回 SUCCESS，失败返回 ERROR
 */
hardware_interface::CallbackReturn ServoSystem::load_configs()
{
  // 1. 读取设备路径（必需参数）
  auto it = info_.hardware_parameters.find("device");
  if (it == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "hardware param 'device' missing");
    return hardware_interface::CallbackReturn::ERROR;
  }
  device_path_ = it->second;

  // 2. 读取 dry_run 参数（可选，默认 false）
  // 干运行模式：在 PC 上测试时不打开真实设备，只打印映射结果
  dry_run_ = false;
  auto dr = info_.hardware_parameters.find("dry_run");
  if (dr != info_.hardware_parameters.end()) {
    const std::string & v = dr->second;
    dry_run_ = (v == "1" || v == "true" || v == "True" || v == "TRUE");
  }

  // 3. 读取 dry_run_print 参数（可选，默认 false）
  // 控制干运行模式下是否打印调试信息（避免日志刷屏）
  dry_run_print_ = false;
  auto drp = info_.hardware_parameters.find("dry_run_print");
  if (drp != info_.hardware_parameters.end()) {
    const std::string & v = drp->second;
    dry_run_print_ = (v == "1" || v == "true" || v == "True" || v == "TRUE");
  }

  // 4. 读取关节配置（必需参数，YAML 字符串格式）
  auto jt = info_.hardware_parameters.find("joints");
  if (jt == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "hardware param 'joints' missing");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 5. 解析 YAML 字符串
  YAML::Node joints_yaml;
  try {
    joints_yaml = YAML::Load(jt->second);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Failed to parse joints YAML: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 6. 验证 YAML 格式（必须是数组）
  if (!joints_yaml.IsSequence()) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "hardware param 'joints' must be a YAML sequence");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 7. 解析每个关节的配置
  joint_cfgs_.clear();
  for (const auto & node : joints_yaml) {
    JointConfig cfg;
    try {
      cfg.name = node["name"].as<std::string>();
      cfg.servo_id = node["servo_id"].as<int>();
      cfg.angle_min = node["angle_min"].as<double>();
      cfg.angle_max = node["angle_max"].as<double>();
      cfg.pulse_min = node["pulse_min_ns"].as<double>();
      cfg.pulse_max = node["pulse_max_ns"].as<double>();
      cfg.pulse_zero = node["pulse_zero_ns"].as<double>();
      cfg.direction = node["direction"].as<int>();
      cfg.offset = node["offset_rad"].as<double>();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Invalid joint entry: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 验证角度范围有效性
    if (std::abs(cfg.angle_max - cfg.angle_min) < 1e-6) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Invalid angle range for joint %s: angle_min == angle_max", cfg.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_cfgs_.push_back(cfg);
  }

  // 8. 验证关节数量是否匹配（配置中的关节数必须等于 URDF 中的关节数）
  if (joint_cfgs_.size() != info_.joints.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Joint config size (%zu) != ros2_control joints (%zu)",
      joint_cfgs_.size(), info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 9. 验证关节名称和顺序是否匹配（配置中的关节名必须与 URDF 中的一致）
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (joint_cfgs_[i].name != info_.joints[i].name) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint order/name mismatch at index %zu: cfg=%s urdf=%s",
        i, joint_cfgs_[i].name.c_str(), info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 10. 打印加载的配置（用于调试）
  for (const auto & cfg : joint_cfgs_) {
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "Joint %s: servo_id=%d angle[%.3f, %.3f] pulse[%.0f, %.0f] zero=%.0f dir=%d offset=%.3f",
      cfg.name.c_str(), cfg.servo_id, cfg.angle_min, cfg.angle_max,
      cfg.pulse_min, cfg.pulse_max, cfg.pulse_zero, cfg.direction, cfg.offset);
  }

  // 11. 打印运行模式信息
  RCLCPP_WARN(
    rclcpp::get_logger(kLoggerName),
    "dry_run=%s dry_run_print=%s (device=%s)",
    dry_run_ ? "true" : "false",
    dry_run_print_ ? "true" : "false",
    device_path_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 打开设备文件
 * 
 * 在干运行模式下直接返回成功（不打开设备）。
 * 在正常模式下打开 /dev/servo_pulse 设备文件，用于写入舵机控制命令。
 * 
 * @return 成功返回 SUCCESS，失败返回 ERROR
 */
hardware_interface::CallbackReturn ServoSystem::open_device()
{
  // 干运行模式：不打开真实设备
  if (dry_run_) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  // 打开设备文件（字符设备，用于写入舵机命令）
  dev_stream_.open(device_path_);
  if (!dev_stream_.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Failed to open %s", device_path_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // 设置输出格式：固定小数点格式（避免科学计数法）
  dev_stream_.setf(std::ios::fmtflags(0), std::ios::floatfield);
  dev_stream_ << std::fixed;
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 关闭设备文件
 * 
 * 如果设备文件已打开，则关闭它。
 */
void ServoSystem::close_device()
{
  if (dev_stream_.is_open()) {
    dev_stream_.close();
  }
}

/**
 * @brief 初始化硬件接口
 * 
 * 1. 调用基类初始化
 * 2. 初始化命令和状态数组（初始值为 0）
 * 3. 加载硬件配置（设备路径、关节参数等）
 * 
 * @param info 硬件信息（从 ros2_control xacro 中读取）
 * @return 成功返回 SUCCESS，失败返回 ERROR
 */
hardware_interface::CallbackReturn ServoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 调用基类初始化（解析 URDF 中的关节信息）
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初始化命令和状态数组（所有关节初始位置为 0）
  const auto n = info_.joints.size();
  cmd_position_.assign(n, 0.0);    // 命令位置数组（弧度）
  state_position_.assign(n, 0.0);  // 状态位置数组（弧度）

  // 加载硬件配置（从 hardware_params.yaml 中读取）
  return load_configs();
}

/**
 * @brief 导出状态接口
 * 
 * 为每个关节创建一个 position 状态接口，供控制器读取当前关节位置。
 * 控制器通过这个接口读取 state_position_[i] 的值。
 * 
 * @return 状态接口列表
 */
std::vector<hardware_interface::StateInterface> ServoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // 为每个关节创建一个 position 状态接口
    // 接口名：关节名，接口类型：HW_IF_POSITION，数据指针：&state_position_[i]
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &state_position_[i]));
  }
  return state_interfaces;
}

/**
 * @brief 导出命令接口
 * 
 * 为每个关节创建一个 position 命令接口，供控制器写入目标关节位置。
 * 控制器通过这个接口写入 cmd_position_[i] 的值。
 * 
 * @return 命令接口列表
 */
std::vector<hardware_interface::CommandInterface> ServoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // 为每个关节创建一个 position 命令接口
    // 接口名：关节名，接口类型：HW_IF_POSITION，数据指针：&cmd_position_[i]
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &cmd_position_[i]));
  }
  return command_interfaces;
}

/**
 * @brief 激活硬件
 * 
 * 1. 初始化状态位置（目前使用伪反馈：直接复制命令值）
 * 2. 打开设备文件
 * 
 * @param previous_state 之前的状态（未使用）
 * @return 成功返回 SUCCESS，失败返回 ERROR
 */
hardware_interface::CallbackReturn ServoSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  // 伪反馈：将命令值复制到状态值（目前没有真实编码器反馈）
  state_position_ = cmd_position_;
  
  // 初始化节流时间戳
  last_write_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
  
  // 打开设备文件
  if (open_device() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 停用硬件
 * 
 * 关闭设备文件。
 * 
 * @param previous_state 之前的状态（未使用）
 * @return 成功返回 SUCCESS
 */
hardware_interface::CallbackReturn ServoSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  close_device();
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 读取硬件状态
 * 
 * 目前实现为伪反馈：直接将命令值复制到状态值。
 * 未来如果有编码器，可以在这里读取真实的位置反馈。
 * 
 * @param time 当前时间（未使用）
 * @param period 控制周期（未使用）
 * @return 成功返回 OK
 */
hardware_interface::return_type ServoSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // 伪反馈：将命令值复制到状态值
  // 这样 MoveIt 和 RViz 可以显示当前关节位置（虽然不一定是真实位置）
  state_position_ = cmd_position_;
  return hardware_interface::return_type::OK;
}

/**
 * @brief 写入命令到硬件
 * 
 * 核心功能：将控制器发送的关节角度命令（弧度）转换为舵机脉宽（纳秒），并写入设备。
 * 
 * 转换流程：
 * 1. 应用方向修正和偏移：a = direction * (cmd + offset)
 * 2. 限制角度范围：a = clamp(a, angle_min, angle_max)
 * 3. 分段线性映射（以 zero 为锚点）：
 *    - 如果 a >= 0：从 pulse_zero 线性插值到 pulse_max
 *    - 如果 a < 0：从 pulse_zero 线性插值到 pulse_min
 * 4. 限制脉宽范围：pulse = clamp(pulse, pulse_min, pulse_max)
 * 5. 写入设备：格式为 "<servo_id> <pulse_ns>\n"
 * 
 * 干运行模式：
 * - 不打开设备，只打印映射结果（如果 dry_run_print 启用）
 * - 采用轮询打印：每 0.5 秒打印一个关节的信息，避免日志刷屏
 * 
 * @param time 当前时间（未使用）
 * @param period 控制周期（未使用）
 * @return 成功返回 OK，失败返回 ERROR
 */
hardware_interface::return_type ServoSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  // 节流机制：确保写入频率与舵机频率匹配（50 Hz = 每 20 ms 一次）
  // 如果距离上次写入时间不足 20 ms，跳过本次写入（避免命令堆积）
  if (!dry_run_) {
    const auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    const int64_t elapsed_ns = (now - last_write_time_).nanoseconds();
    if (elapsed_ns < SERVO_UPDATE_PERIOD_NS) {
      // 时间间隔太短，跳过本次写入（舵机还没准备好接收新命令）
      return hardware_interface::return_type::OK;
    }
    last_write_time_ = now;
  }
  
  // 用于干运行模式下的日志打印（静态变量，保持状态）
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  static rclcpp::Time last_print = steady_clock.now();
  static size_t print_idx = 0;  // 轮询打印的索引
  static std::vector<double> last_cmd_position;  // 上次的命令位置（用于检测变化）
  if (last_cmd_position.size() != cmd_position_.size()) {
    last_cmd_position = cmd_position_;
  }

  // 遍历所有关节，将角度命令转换为脉宽并写入设备
  for (size_t i = 0; i < joint_cfgs_.size(); ++i) {
    const auto & cfg = joint_cfgs_[i];
    
    // ========== 步骤 1：应用方向修正和偏移 ==========
    // direction: 1 表示正方向，-1 表示反向（用于处理舵机安装方向相反的情况）
    // offset: 角度偏移（用于机械零点校准）
    double a = cfg.direction * (cmd_position_[i] + cfg.offset);
    
    // ========== 步骤 2：限制角度范围 ==========
    a = clamp(a, cfg.angle_min, cfg.angle_max);
    
    // ========== 步骤 3：分段线性映射（以 zero 为锚点） ==========
    // 关键：确保 angle=0 时，pulse=pulse_zero
    double pulse = cfg.pulse_zero;
    
    if (a >= 0.0) {
      // 正角度：从 pulse_zero 线性插值到 pulse_max
      // 公式：pulse = pulse_zero + a * (pulse_max - pulse_zero) / angle_max
      double denom = cfg.angle_max;
      if (std::abs(denom) < 1e-6) {
        // 如果 angle_max 为 0，使用总范围作为分母
        denom = cfg.angle_max - cfg.angle_min;
      }
      pulse = cfg.pulse_zero + a * (cfg.pulse_max - cfg.pulse_zero) / denom;
    } else {
      // 负角度：从 pulse_zero 线性插值到 pulse_min
      // 公式：pulse = pulse_zero + a * (pulse_zero - pulse_min) / (-angle_min)
      // 注意：a 是负数，所以这个公式会让 pulse 从 pulse_zero 向 pulse_min 减小
      double denom = -cfg.angle_min;
      if (std::abs(denom) < 1e-6) {
        // 如果 angle_min 为 0，使用总范围作为分母
        denom = cfg.angle_max - cfg.angle_min;
      }
      pulse = cfg.pulse_zero + a * (cfg.pulse_zero - cfg.pulse_min) / denom;
    }
    
    // ========== 步骤 4：限制脉宽范围 ==========
    pulse = clamp(pulse, cfg.pulse_min, cfg.pulse_max);

    // ========== 干运行模式：只打印，不写入设备 ==========
    if (dry_run_) {
      // 只有在启用 dry_run_print 时才打印
      if (dry_run_print_) {
        const auto now = steady_clock.now();
        
        // 检测是否有命令变化（避免无变化时也打印）
        bool any_changed = false;
        for (size_t k = 0; k < cmd_position_.size(); ++k) {
          if (std::abs(cmd_position_[k] - last_cmd_position[k]) > 1e-4) {
            any_changed = true;
            break;
          }
        }
        
        // 轮询打印：每 0.5 秒打印一个关节的信息（避免日志刷屏）
        if (any_changed && (now - last_print).nanoseconds() > 500000000LL && !joint_cfgs_.empty()) {
          const size_t j = print_idx % joint_cfgs_.size();  // 轮询选择关节
          const auto & pcfg = joint_cfgs_[j];
          
          // 重新计算该关节的映射（用于打印）
          double pa = pcfg.direction * (cmd_position_[j] + pcfg.offset);
          pa = clamp(pa, pcfg.angle_min, pcfg.angle_max);
          double ppulse = pcfg.pulse_zero;
          if (pa >= 0.0) {
            double denom = pcfg.angle_max;
            if (std::abs(denom) < 1e-6) {
              denom = pcfg.angle_max - pcfg.angle_min;
            }
            ppulse = pcfg.pulse_zero + pa * (pcfg.pulse_max - pcfg.pulse_zero) / denom;
          } else {
            double denom = -pcfg.angle_min;
            if (std::abs(denom) < 1e-6) {
              denom = pcfg.angle_max - pcfg.angle_min;
            }
            ppulse = pcfg.pulse_zero + pa * (pcfg.pulse_zero - pcfg.pulse_min) / denom;
          }
          ppulse = clamp(ppulse, pcfg.pulse_min, pcfg.pulse_max);

          // 打印调试信息
          RCLCPP_INFO(
            rclcpp::get_logger(kLoggerName),
            "DRY_RUN: %s cmd=%.3f -> servo_id=%d pulse_ns=%.0f",
            pcfg.name.c_str(), cmd_position_[j], pcfg.servo_id, ppulse);

          last_print = now;
          print_idx++;  // 下次打印下一个关节
          last_cmd_position = cmd_position_;
        }
      }
      continue;  // 干运行模式：跳过设备写入
    }

    // ========== 正常模式：写入设备 ==========
    if (!dev_stream_.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Device stream not open");
      return hardware_interface::return_type::ERROR;
    }

    // 写入设备：格式为 "<servo_id> <pulse_ns>\n"
    // 例如："0 1500000\n" 表示舵机 0 的脉宽为 1500000 纳秒（1.5 毫秒）
    dev_stream_ << cfg.servo_id << " " << static_cast<uint64_t>(pulse) << "\n";
    if (!dev_stream_) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Failed writing to device");
      return hardware_interface::return_type::ERROR;
    }
    dev_stream_.flush();  // 立即刷新缓冲区，确保命令及时发送
  }
  return hardware_interface::return_type::OK;
}

}  // namespace my_arm_hw

PLUGINLIB_EXPORT_CLASS(my_arm_hw::ServoSystem, hardware_interface::SystemInterface)

