#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "arm_control_sdk/data_type_def.h"

namespace carm {
class CArmKernelImpl;
class CArmSingleCol {
public:
    /**
     * @brief 构造函数, 支持A3（六轴）系列单臂, 抛出 throw std::runtime_error
     * 异常如果连接的机械臂不是A3系列
     *
     * @param server_ip
     * @param port
     * @param timeout 连接超时，单位秒
     */
    CArmSingleCol(const std::string& server_ip = "10.42.0.101",
                  int port = 8090,
                  double timeout = 1) noexcept(false);
    ~CArmSingleCol();
    /**
     * @brief 连接carm controller，抛出 throw std::runtime_error 异常如果连接的机械臂不是A3系列
     *
     * @param server_ip
     * @param port
     * @param timeout 连接超时，单位秒
     * @return int 1: 连接成功，<1:连接失败
     */
    int connect(const std::string& server_ip = "10.42.0.101",
                int port = 8090,
                double timeout = 1) noexcept(false);
    /**
     * @brief disconnect with carm
     * @return int 1: 断连成功，-1: 断连失败
     */
    int disconnect();
    /**
     * @brief 判断是否连接carm controller
     *
     * @return true 已连接
     * @return false 未连接
     */
    bool is_connected();

    /*******************基础函数******************* */
    /**
     * @brief 组合指令，控制器复位
     * clean_carm_error();
     * set_servo_enable();
     * set_control_mode(0);
     * @return int 1: 复位成功，-1: 复位失败
     */
    int set_ready();

    /**
     * @brief Set the servo enable object
     *
     * @param enable 设置伺服上使能(true) or 下使能(false)
     * @return int 非阻塞，1: 指令发送成功，<1: 指令发送失败
     */
    int set_servo_enable(bool enable);

    /**
     * @brief 设置控制器控制（fsm）模式
     *
     * @param mode
     * 0-IDLE空闲模式
     * 1-点位控制模式
     * 2-MIT控制模式
     * 3-关节拖动模式
     * 4-PF力位混合模式
     * @return int 非阻塞，1: 指令发送成功，<1: 指令发送失败
     */
    int set_control_mode(int mode);

    /**
     * @brief set passthrough data
     * @param mode 模式 0-发送 1-接收 2-发送并接收
     * @param can_id CAN ID 发送输入，同时也返回读取值
     * @param data 数据 发送输入，同时也可以返回读取值
     * @return int 非阻塞，1: 指令发送成功，<1: 指令发送失败
     */
    int set_passthrough_data(int mode, uint32_t& can_id, std::vector<uint8_t>& data);

    /**
     * @brief 通过 EtherCAT 透传板发送或接收完整 CAN/CAN FD 帧
     * @param mode 0-发送，1-接收，2-发送并等待响应
     * @param frame 输入帧；成功后替换为后端返回帧
     * @param timeout_ms mode=2 时等待设备响应的超时时间，单位毫秒
     * @return 1-成功，-1-SDK 通信失败，-2-后端拒绝或执行失败，-3-后端响应格式异常
     */
    int set_ecat_passthrough_data(int mode, EcatCanFrame& frame, uint32_t timeout_ms = 100);

    /**
     * @brief 获取控制器软件版本
     *
     * @return std::string
     */
    std::string get_version();
    /**
     * @brief 获取主要配置参数，包括关节限位、关节最大速度、加速度、加加速度等
     *
     * @return ArmConfig
     */
    ArmConfig get_config();

    /**
     * @brief 获取末端执行器配置
     *
     * @return EndEffectorConfig
     */
    EndEffectorConfig get_eeff_config();

    /**
     * @brief 获取机械臂状态
     *
     * @return ArmStatus
     */
    ArmStatus get_status();
    /**
     * @brief 获取实际的关节角度
     *
     * @return std::vector<double>
     */
    std::vector<double> get_joint_pos();
    /**
     * @brief 获取实际的关节角速度
     *
     * @return std::vector<double>
     */
    std::vector<double> get_joint_vel();
    /**
     * @brief 获取实际的关节力矩
     *
     * @return std::vector<double>
     */
    std::vector<double> get_joint_tau();
    /**
     * @brief 获取控制关节指令的关节角度
     *
     * @return std::vector<double>
     */
    std::vector<double> get_plan_joint_pos();
    /**
     * @brief 获取控制关节指令的关节角速度
     *
     * @return std::vector<double>
     */
    std::vector<double> get_plan_joint_vel();
    /**
     * @brief 获取控制关节指令的关节力矩
     *
     * @return std::vector<double>
     */
    std::vector<double> get_plan_joint_tau();
    /**
     * @brief 获取控制法兰相对基座的位姿
     *
     * @return std::array<double, 7>: 位姿包括位置和四元数（x, y, z, qx, qy, qz, qw）
     */
    std::array<double, 7> get_plan_cart_pose();
    /**
     * @brief 获取实际法兰相对基座的位姿
     *
     * @return std::array<double, 7>: 位姿包括位置和四元数（x, y, z, qx, qy, qz, qw）
     */
    std::array<double, 7> get_cart_pose();
    /**
     * @brief 关节进行重力补偿后受到的其他力矩
     *
     * @return std::vector<double>
     */
    std::vector<double> get_joint_external_tau();
    /**
     * @brief 获取末端力控的力矩
     *
     * @return std::vector<double>: 笛卡尔外力和力矩（fx, fy, fz, tx, ty, tz）
     */
    std::vector<double> get_cart_external_force();
    /**
     * @brief 注册实时更新的带时间戳的关节信息
     *
     * @param cbk 回调函数，接收参数（time, joints_pos, joints_vel, joints_tau）
     */
    void register_joint_cbk(
            std::function<
                    void(double, std::vector<double>, std::vector<double>, std::vector<double>)>
                    cbk);
    void release_joint_cbk();
    /**
     * @brief 注册实时更新的带时间戳的末端信息
     *
     * @param cbk 回调函数，接收参数（time, pose），位姿包括位置和四元数（x, y, z, qx, qy, qz, qw）
     */
    void register_pose_cbk(std::function<void(double, std::array<double, 7>)> cbk);
    void release_pose_cbk();
    /**
     * @brief 注册实时更新的带时间戳的关节控制指令信息
     *
     * @param cbk 回调函数，接收参数（time, cmd_pos, cmd_vel, cmd_tau）
     */
    void register_plan_joint_cbk(
            std::function<
                    void(double, std::vector<double>, std::vector<double>, std::vector<double>)>
                    cbk);
    void release_plan_joint_cbk();
    /**
     * @brief 注册实时更新的带时间戳的末端控制指令信息
     *
     * @param cbk 回调函数，接收参数（time, pose），位姿包括位置和四元数（x, y, z, qx, qy, qz, qw）
     */
    void register_plan_pose_cbk(std::function<void(double, std::array<double, 7>)> cbk);
    void release_plan_pose_cbk();
    /**
     * @brief 注册实时更新的带时间戳的外力矩信息
     *
     * @param cbk 回调函数，接收参数（time, joints_tau, cart_external_force）
     */
    void register_external_force_cbk(
            std::function<void(double, std::vector<double>, std::vector<double>)> cbk);
    void release_external_force_cbk();

    /*################end effector##############*/
    /**
     * @brief 获取末端执行器状态
     *
     * @return int: -1: 未连接， 0: 未使能, 1: 正常状态, >1: 对应伺服错误
     */
    int get_eeff_state();
    /**
     * @brief 获取末端执行器位置
     *
     * @return std::vector<double>: 末端执行器位置
     */
    std::vector<double> get_eeff_pos();
    /**
     * @brief 获取末端执行器速度
     *
     * @return std::vector<double>: 末端执行器速度
     */
    std::vector<double> get_eeff_vel();
    /**
     * @brief 获取末端执行器力矩
     *
     * @return std::vector<double>: 末端执行器力矩
     */
    std::vector<double> get_eeff_tau();
    /**
     * @brief 获取末端执行器电机位置
     *
     * @return std::vector<double>: 末端执行器电机位置
     */
    std::vector<double> get_eeff_motor_pos();
    /**
     * @brief 获取末端执行器电机速度
     *
     * @return std::vector<double>: 末端执行器电机速度
     */
    std::vector<double> get_eeff_motor_vel();
    /**
     * @brief 获取末端执行器电机力矩
     *
     * @return std::vector<double>: 末端执行器力矩
     */
    std::vector<double> get_eeff_motor_tau();
    /**
     * @brief 获取末端执行器规划位置
     *
     * @return std::vector<double>: 末端执行器规划位置
     */
    std::vector<double> get_plan_eeff_pos();
    /**
     * @brief 获取末端执行器规划速度
     *
     * @return std::vector<double>: 末端执行器规划速度
     */
    std::vector<double> get_plan_eeff_vel();
    /**
     * @brief 获取末端执行器规划力矩
     *
     * @return std::vector<double>: 末端执行器规划力矩
     */
    std::vector<double> get_plan_eeff_tau();
    /**
     * @brief 获取末端执行器电机规划位置
     *
     * @return std::vector<double>: 末端执行器规划力矩
     */
    std::vector<double> get_plan_eeff_motor_pos();
    /**
     * @brief 获取末端执行器电机规划速度
     *
     * @return std::vector<double>: 末端执行器规划力矩
     */
    std::vector<double> get_plan_eeff_motor_vel();
    /**
     * @brief 获取末端执行器电机规划力矩
     *
     * @return std::vector<double>: 末端执行器规划力矩
     */
    std::vector<double> get_plan_eeff_motor_tau();
    /**
     * @brief 获取末端执行器类型
     *
     * @return std::string: "gripper" / "hand" / "flange" / 自定义类型
     */
    std::string get_eeff_type();
    /**
     * @brief 获取末端执行器自由度
     *
     * @return int: 末端执行器自由度
     */
    int get_eeff_dof();
    /**
     * @brief 获取末端执行器连接状态
     *
     * @return bool: true 表示已连接，false 表示未连接
     */
    bool get_eeff_connect();

    // ---- deprecated: 旧接口，内部转发到 eeff_ 等价方法 ----
    /** @deprecated use get_eeff_state() */
    int get_gripper_state();
    /** @deprecated use get_eeff_pos() */
    double get_gripper_pos();
    /** @deprecated use get_eeff_vel() */
    double get_gripper_vel();
    /** @deprecated use get_eeff_tau() */
    double get_gripper_tau();
    /** @deprecated use get_plan_eeff_pos() */
    double get_plan_gripper_pos();
    /** @deprecated use get_plan_eeff_tau() */
    double get_plan_gripper_tau();
    /** @deprecated use get_eeff_state() */
    int get_hand_state();
    /** @deprecated use get_eeff_pos() */
    std::vector<double> get_hand_pos();
    /** @deprecated use get_eeff_vel() */
    std::vector<double> get_hand_vel();
    /** @deprecated use get_eeff_tau() */
    std::vector<double> get_hand_tau();
    /** @deprecated use get_plan_eeff_pos() */
    std::vector<double> get_plan_hand_pos();
    /** @deprecated use get_plan_eeff_vel() */
    std::vector<double> get_plan_hand_vel();
    /** @deprecated use get_plan_eeff_tau() */
    std::vector<double> get_plan_hand_tau();
    /*******************运动函数******************* */
    /**
     * @brief 跟随运动，周期性发送目标关节位置
     *
     * @param targets 目标关节位置列表
     * @param eeff_pos 末端执行器位置（夹爪两指间隔，dof=1）
     * @return int 非阻塞，1: 指令发送成功，<1: 指令发送失败
     */
    int track_joint(const std::vector<double>& targets, const double eeff_pos = -1);
    /**
     * @brief 跟随运动，周期性发送目标位姿，法兰相对基座
     *
     * @param targets 目标位姿，法兰相对基座（x, y, z, qx, qy, qz, qw）
     * @param eeff_pos 末端执行器位置（夹爪两指间隔，dof=1）
     * @return int 非阻塞，1: 指令发送成功，<1: 指令发送失败
     */
    int track_pose(const std::array<double, 7>& targets, const double eeff_pos = -1);
    /**
     * @brief 关节到点运动
     *
     * @param target_pos 关节目标位置
     * @param desire_time 目标到达时间，默认-1表示没有时间要求
     * @param is_sync 是否同步，true: 接口阻塞至任务完成 false: 接口非阻塞
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int move_joint(const std::vector<double>& target_pos,
                   double desire_time = -1,
                   bool is_sync = true);
    /**
     * @brief 关节到点运动
     *
     * @param target_pos 目标位姿，法兰相对基座
     * @param desire_time 目标到达时间，默认-1表示没有时间要求
     * @param is_sync 是否同步，true: 接口阻塞至任务完成 false: 接口非阻塞
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int move_pose(const std::array<double, 7>& target_pos,
                  double desire_time = -1,
                  bool is_sync = true);

    /**
     * @brief 直线到点运动
     *
     * @param target_pos 关节目标位置
     * @param is_sync 是否同步，true: 接口阻塞至任务完成 false: 接口非阻塞
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int move_line_joint(const std::vector<double>& target_pos, bool is_sync = true);
    /**
     * @brief 直线到点运动
     *
     * @param target_pos 目标位姿，法兰相对基座
     * @param is_sync 是否同步，true: 接口阻塞至任务完成 false: 接口非阻塞
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int move_line_pose(const std::array<double, 7>& target_pos, bool is_sync = true);

    /**
     * @brief PT运动
     *
     * @param target_pos 目标关节位置
     * @param eeff_pos 目标末端执行器位置，默认为空表示没有末端控制要求
     * @param stamps 时间戳，默认为空表示没有时间要求
     * @param is_sync 是否同步，true: 接口阻塞至任务完成 false: 接口非阻塞
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int move_joint_traj(const std::vector<std::vector<double>>& target_pos,
                        const std::vector<double> eeff_pos = {},
                        std::vector<double> stamps = {},
                        bool is_sync = true);
    /**
     * @brief PT运动
     *
     * @param target_pos 目标位姿，法兰相对基座
     * @param eeff_pos 目标末端执行器位置，默认为空表示没有末端控制要求
     * @param stamps 时间戳, 默认为空表示没有时间要求
     * @param is_sync 是否同步，true: 接口阻塞至任务完成 false: 接口非阻塞
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int move_pose_traj(const std::vector<std::array<double, 7>>& target_pos,
                       const std::vector<double> eeff_pos = {},
                       std::vector<double> stamps = {},
                       bool is_sync = true);
    /**
     * @brief 位姿迭代运动
     *
     * @param target_pos 目标位姿，法兰相对基座
     * @param line_theta_weight 位置和姿态的权重
     * @param accuracy 精度
     * @param is_sync 是否同步，true: 接口阻塞至任务完成 false: 接口非阻塞
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */

    int move_flow_pose(const std::array<double, 7>& target_pos,
                       double line_theta_weight = 0.5,
                       double accuracy = 0.0001,
                       bool is_sync = true);

    /**
     * @brief 急停，恢复需要调set_ready接口
     *
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int emergency_stop();
    /**
     * @brief 设置控制器进入debug仿真模式，该模式下不连接机械臂
     *
     * @param flag
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int set_debug(bool flag);

    /**
     * @brief 停止任务
     *
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int task_stop();

    /**
     * @brief 末端执行器统一控制
     *
     * @param pos 末端执行器位置
     * @param vel 末端执行器速度，默认为 0
     * @param tau 末端执行器力矩
     * @param control_motor true: 控制末端电机并使用 motor_* 限制；false: 控制末端执行器
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int set_eeff(const std::vector<double>& pos,
                 const std::vector<double>& vel,
                 const std::vector<double>& tau,
                 bool control_motor = false);

    // ---- deprecated: 旧控制接口，内部转发到 set_eeff() ----
    /** @deprecated use set_eeff(pos, vel, tau) */
    int set_gripper(double pos, double tau = 10);
    /** @deprecated use set_eeff(pos, vel, tau) */
    int set_hand(const std::vector<double>& pos,
                 const std::vector<double>& tau,
                 const std::vector<double>& vel);

    /*******************设置函数******************* */
    /**
     * @brief 在线改变速度等级（在线降速）
     *
     * @param level 速度等级 0-10分别对应速度百分比0%-100%
     * @param response_level 响应等级
     * 插补周期数，表示在多少个周期内完成速度转变，范围1-100
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int set_speed_level(double level, int response_level = 20);

    /**
     * @brief 设置拖动模式参数
     *
     * @param torque_factor 转矩因子，范围[0, 2]，数值越大拖动感觉越轻，建议范围1-1.2
     * @param friction_compensation_factor 摩擦力补偿系数，范围[0,
     * 2]，建议范围0.5-1.2
     *
     */
    int set_drag_params(std::vector<double> torque_factor,
                        std::vector<double> friction_compensation_factor);

    /**
     * @brief 设置当前工具号
     *
     * @param index
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int set_tool_index(int index);
    /**
     * @brief 获取当前工具号
     *
     * @return int
     */
    int get_tool_index();
    /**
     * @brief 获取指定工具的坐标系（工具末端相对法兰的位姿关系）
     *
     * @param index
     * @return std::array<double, 7>
     */
    std::array<double, 7> get_tool_coordinate(int index);

    /**
     * @brief 启动/关闭碰撞检测，设置碰撞检测等级
     *
     * @param enable_flag 使能，是否开启碰撞检测
     * @param sensitivity_level 灵敏度等级 0-2，0最高
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int set_collision_config(bool enable_flag = true, int sensitivity_level = 0);

    /**
     * @brief 开始/停止示教录制
     *
     * @param off_on true: 开始示教轨迹，false: 停止轨迹记录
     * @param name 路径的命名（仅在开始示教记录时有效）
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int trajectory_teach(bool off_on, std::string name);
    /**
     * @brief 复现指定名称的轨迹
     *
     * @param name 轨迹名称
     * @param is_sync 是否同步等待复现完成，默认true
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int trajectory_recorder(std::string name, bool is_sync = true);
    /**
     * @brief 获取已记录的示教轨迹列表
     *
     * @param traj_list 引用传递，在此参数中返回获取到的轨迹名称列表
     * @return int 1: 指令发送成功，<1: 指令发送失败
     */
    int check_teach(std::vector<std::string>& traj_list);

    /**
     * @brief 批量逆运动学求解
     *
     * @param tool_index 工具号
     * @param quat_pose 目标位姿列表（x, y, z, qx, qy, qz, qw）
     * @param ref_joint 参考关节角度列表，用于选取多解时的最优解
     * @param jnt_value 输出的关节角度结果列表
     * @return int 1: 求解成功，<1: 求解失败
     */
    int inverse_kine_array(int tool_index,
                           const std::vector<std::array<double, 7>>& quat_pose,
                           const std::vector<std::vector<double>>& ref_joint,
                           std::vector<std::vector<double>>& jnt_value);
    /**
     * @brief 批量正运动学求解
     *
     * @param tool_index 工具号
     * @param jnt_value 关节角度列表
     * @param quat_pose 输出的位姿结果列表（x, y, z, qx, qy, qz, qw）
     * @return int 1: 求解成功，<1: 求解失败
     */
    int forward_kine_array(int tool_index,
                           const std::vector<std::vector<double>>& jnt_value,
                           std::vector<std::array<double, 7>>& quat_pose);
    /**
     * @brief 单点逆运动学求解
     *
     * @param tool_index 工具号
     * @param quat_pose 目标位姿（x, y, z, qx, qy, qz, qw）
     * @param ref_joint 参考关节角度，用于选取多解时的最优解
     * @param jnt_value 输出的关节角度结果
     * @return int 1: 求解成功，<1: 求解失败
     */
    int inverse_kine(int tool_index,
                     const std::array<double, 7>& quat_pose,
                     const std::vector<double>& ref_joint,
                     std::vector<double>& jnt_value);
    /**
     * @brief 单点正运动学求解
     *
     * @param tool_index 工具号
     * @param jnt_value 关节角度
     * @param quat_pose 输出的位姿结果（x, y, z, qx, qy, qz, qw）
     * @return int 1: 求解成功，<1: 求解失败
     */
    int forward_kine(int tool_index,
                     const std::vector<double>& jnt_value,
                     std::array<double, 7>& quat_pose);

    /**
     * @brief 注册错误回调函数
     *
     * @param key 标识回调的键值
     * @param cbk 回调函数，接收参数（错误码，错误信息字符串）
     */
    void register_error_cbk(const std::string& key,
                            std::function<void(int, const std::string)> cbk);
    /**
     * @brief 注销错误回调函数
     *
     * @param key 要注销的回调键值
     */
    void release_error_cbk(const std::string& key);

    /**
     * @brief 注册任务完成回调函数
     *
     * @param key 标识回调的键值
     * @param cbk 回调函数，接收参数（任务task_key字符串）
     */
    void register_completion_cbk(const std::string& key,
                                 std::function<void(const std::string)> cbk);
    /**
     * @brief 注销任务完成回调函数
     *
     * @param key 要注销的回调键值
     */
    void release_completion_cbk(const std::string& key);

    /*******************底层透传接口******************* */
    /**
     * @brief 设置底层透传模式
     *
     * @param flag true开启底层透传模式，false关闭
     * @return int 1表示成功，<0表示失败
     */
    int set_low_mode(bool flag);

    /**
     * @brief 发送底层PV控制指令并获取硬件状态
     */
    int low_pv_command(const std::vector<double>& pos,
                       const std::vector<double>& vel,
                       RobotLowData& data);

    /**
     * @brief 发送底层MIT控制指令并获取硬件状态
     */
    int low_mit_command(const std::vector<double>& pos,
                        const std::vector<double>& vel,
                        const std::vector<double>& tau,
                        const std::vector<double>& kp,
                        const std::vector<double>& kd,
                        RobotLowData& data);

    /**
     * @brief 发送底层PF控制指令并获取硬件状态
     */
    int low_pf_command(const std::vector<double>& pos,
                       const std::vector<double>& vel,
                       const std::vector<double>& tau,
                       RobotLowData& data);

    /**
     * @brief 发送底层Current控制指令并获取硬件状态
     */
    int low_current_command(const std::vector<double>& tau, RobotLowData& data);

    /**
     * @brief 主动刷新并获取底层硬件数据
     */
    int low_refresh(RobotLowData& data);

    /**
     * @brief 底层末端执行器控制指令
     */
    int low_set_end_effector_ctr(const std::vector<double>& pos,
                                 const std::vector<double>& vel,
                                 const std::vector<double>& tau,
                                 RobotLowData& data);

    /**
     * @brief 设置机器人底层运行模式
     */
    int low_set_robot_mode(int mode);

    /**
     * @brief 设置末端执行器底层运行模式
     */
    int low_set_end_effector_mode(int mode);

    /**
     * @brief 控制底层伺服上/下使能
     */
    int low_set_servo_enable(bool status);

    /**
     * @brief 底层错误复位
     *
     * @param cnt 尝试复位的最大次数，默认5次
     */
    int low_reset(int cnt = 5);

    /**
     * @brief 获取底层伺服级状态
     */
    int low_get_servo_status(ServoStatus& status);

    /**
     * @brief 底层逆运动学求解
     */
    int low_get_inverse_kine(const std::array<double, 7>& pose,
                             const std::vector<double>& refer_pos,
                             std::vector<double>& jnt_value,
                             int tool = -1);

    /**
     * @brief 底层正运动学求解
     */
    int low_get_forward_kine(const std::vector<double>& joint_pos,
                             std::array<double, 7>& quat_pose,
                             int tool = -1);

    /**
     * @brief 底层动力学计算（惯性矩阵、科氏力、重力）
     */
    int low_get_dynamics(const std::vector<double>& joint_pos,
                         const std::vector<double>& joint_vel,
                         const std::vector<double>& joint_acc,
                         int& tool,
                         std::vector<double>& m_force,
                         std::vector<double>& c_force,
                         std::vector<double>& g_force);

    /**
     * @brief 获取底层雅可比矩阵
     */
    int low_get_jacobian(const std::vector<double>& joint_pos, int& tool, RobotMatrix& mat);

    /**
     * @brief 获取底层零空间矩阵
     */
    int low_get_nullspace(const std::vector<double>& joint_pos,
                          double tolerance,
                          int& tool,
                          RobotMatrix& mat);

private:
    bool _is_specified_arm() const;

    CArmKernelImpl* implement_;
};
}  // namespace carm
