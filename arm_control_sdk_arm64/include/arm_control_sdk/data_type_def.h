#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>
namespace carm {
/*******************************
 * 机器运行模式，以及运行状态
 */
enum class CarmState { Carm_Error = -1, Carm_Standby = 0, Carm_Running = 1 };
enum class FsmMode {
    ERROR = -1,     // 错误，异常
    IDLE,           // 空闲状态
    POSITION,       // 点位控制模式
    MIT,            // MIT控制模式
    CURRENT,        // 电流控制
    PF,             // 力位混合模式
    TELEOPERATION,  // 遥操作模式
    IMPEDANCE       // 阻抗控制模式
};
static std::map<FsmMode, std::string> fsm_mode_str_map{{FsmMode::ERROR, "ERROR"},
                                                       {FsmMode::IDLE, "IDLE"},
                                                       {FsmMode::POSITION, "POSITION"},
                                                       {FsmMode::CURRENT, "CURRENT"},
                                                       {FsmMode::MIT, "MIT"},
                                                       {FsmMode::TELEOPERATION, "TELEOPERATION"},
                                                       {FsmMode::PF, "PF"},
                                                       {FsmMode::IMPEDANCE, "IMPEDANCE"}};

static std::map<std::string, FsmMode> str_map_fsm_mode{{"ERROR", FsmMode::ERROR},
                                                       {"IDLE", FsmMode::IDLE},
                                                       {"POSITION", FsmMode::POSITION},
                                                       {"CURRENT", FsmMode::CURRENT},
                                                       {"MIT", FsmMode::MIT},
                                                       {"TELEOPERATION", FsmMode::TELEOPERATION},
                                                       {"PF", FsmMode::PF},
                                                       {"IMPEDANCE", FsmMode::IMPEDANCE}};

static std::map<CarmState, std::string> carm_state_str{{CarmState::Carm_Error, "Carm_Error"},
                                                       {CarmState::Carm_Standby, "Carm_Standby"},
                                                       {CarmState::Carm_Running, "Carm_Running"}};
/*******************************
 * 机械臂固有变量（py支持）
 */
struct ArmConfig {
    ArmConfig(int df) {
        dof = df;
        limit_upper.resize(dof);
        limit_lower.resize(dof);

        joint_vel.resize(dof);
        joint_acc.resize(dof);
        joint_dec.resize(dof);
        joint_jerk.resize(dof);
    }
    ArmConfig() = default;
    // 关节数量
    int dof = 0;
    // 关节限位
    std::vector<double> limit_upper;
    std::vector<double> limit_lower;
    // 关节最大速度、加速度、加加速
    std::vector<double> joint_vel;
    std::vector<double> joint_acc;
    std::vector<double> joint_dec;
    std::vector<double> joint_jerk;
};

struct EndEffectorConfig {
    EndEffectorConfig(int df) {
        dof = df;
        eeff_name = "";
        eeff_type = "";
        limit_upper.resize(dof);
        limit_lower.resize(dof);
        eeff_vel.resize(dof);
        eeff_tau.resize(dof);

        motor_limit_upper.resize(dof);
        motor_limit_lower.resize(dof);
        motor_vel.resize(dof);
        motor_tau.resize(dof);
    }
    EndEffectorConfig() = default;
    // 关节数量
    int dof = 0;
    std::string eeff_name;  // 末端执行器名称
    std::string eeff_type;  // 末端执行器类型，如夹爪、吸盘等
    // 末端执行器限位
    std::vector<double> limit_upper;
    std::vector<double> limit_lower;
    // 末端执行器最大速度、力矩
    std::vector<double> eeff_vel;
    std::vector<double> eeff_tau;

    // 末端执行器模组限位
    std::vector<double> motor_limit_upper;
    std::vector<double> motor_limit_lower;
    // 末端执行器模组最大速度、力矩
    std::vector<double> motor_vel;
    std::vector<double> motor_tau;
};

inline constexpr uint8_t ECAT_CAN_FLAG_EXTENDED = 1U << 0;  // 扩展帧
inline constexpr uint8_t ECAT_CAN_FLAG_REMOTE = 1U << 1;    // 远程帧，仅经典 CAN
inline constexpr uint8_t ECAT_CAN_FLAG_BRS = 1U << 3;       // 波特率切换，仅 CAN FD
inline constexpr uint8_t ECAT_CAN_FLAG_ESI = 1U << 4;       // 错误状态指示，仅 CAN FD

// EtherCAT 透传使用的完整 CAN/CAN FD 帧，数据长度由 data.size() 表示。
struct EcatCanFrame {
    uint32_t can_id = 0;
    uint8_t flags = 0;
    bool can_fd = false;
    std::vector<uint8_t> data;
};

/*******************************
 * 机械臂状态量（py支持）
 */
struct ArmStatus {
    int arm_index = 0;              // 机械臂编号
    std::string arm_name;           // 臂名称
    bool arm_is_connected = false;  // 通过通讯进行控制时的标志
    int arm_dof = 0;                // 臂自由度
    bool servo_status = false;      // 伺服状态 1使能 0失能
    int state = 0;                  // 0-standby,1-running,-1-error 控制器状态
    int fsm_state = 0;              // 0-idle,1-position，2-MIT, 3-drag 控制器模式
    double speed_percentage = 0;    // 速度百分比，通过set_speed_level()改
    bool on_debug_mode = false;     // 是否在仿真状态
};

struct RobotLowStatus {
    int error_code = 0;
    std::string error_msg = "";
    bool arm_connected = false;
    bool arm_enable = false;
    bool arm_status = false;
    int8_t arm_mode = -1;

    int gripper_error_code = 0;
    std::string gripper_error_msg = "";
    bool gripper_connected = false;
    bool gripper_enable = false;
    bool gripper_status = false;
    int8_t gripper_mode = -1;
};

struct RobotLowData {
    RobotLowStatus status;
    std::vector<double> joint_cmd_pos;
    std::vector<double> joint_cmd_vel;
    std::vector<double> joint_cmd_tau;
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
    std::vector<double> joint_tau;

    double gripper_cmd_pos;
    double gripper_cmd_vel;
    double gripper_cmd_tau;
    double gripper_pos;
    double gripper_vel;
    double gripper_tau;
};

struct ServoStatus {
    int dof;
    std::vector<double> mitKp;
    std::vector<double> mitKd;
    std::vector<double> pvVel;
    std::vector<double> pfVel;

    std::vector<uint16_t> motorErrorCode;
    std::vector<std::string> motorErrorMsg;

    std::vector<bool> isServoEnable;
    std::vector<int8_t> fsmMode;
    std::vector<bool> isConnected;

    std::vector<double> mosTemperature;
    std::vector<double> motorTemperature;
    std::vector<double> motorVBus;
};

struct RobotMatrix {
    int rows = 0;
    int cols = 0;
    std::vector<double> data;
};

}  // namespace carm
