#pragma once

#include <cstring>
#include <cstdint>
#include "./hardware/common_gamepad.h"

namespace common_gamepad_wireless
{

typedef union
{
    struct
    {
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;

        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;

        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
    } components;
    uint16_t value;
} KeySwitchUnion;


// ============================================================
// 2. 40 字节结构
//
// byte 0-1   : head
// byte 2-3   : buttons
// byte 4-7   : lx
// byte 8-11  : rx
// byte 12-15 : ry
// byte 16-19 : lt
// byte 20-23 : ly
// byte 24-27 : rt
// byte 28-31 : custom_buttons
// byte 32-35 : mocap_buttons
// byte 36-39 : reserved
// ============================================================

typedef struct
{
    uint8_t head[2];        // 2 bytes
    KeySwitchUnion btn;     // 2 bytes

    float lx;               // 4 bytes
    float rx;               // 4 bytes
    float ry;               // 4 bytes
    float lt;               // 4 bytes
    float ly;               // 4 bytes

    uint8_t idle[16];       // 16 bytes

} RockerBtnDataStruct;


// ============================================================
// 3. union：同一块 40 字节内存，可以用结构体访问，也可以用 buff[40] 访问
// ============================================================

typedef union
{
    RockerBtnDataStruct data;
    uint8_t buff[40];

} RemoteData;


// 确保大小一定是 40 字节
static_assert(sizeof(KeySwitchUnion) == 2, "KeySwitchUnion must be 2 bytes");
static_assert(sizeof(RockerBtnDataStruct) == 40, "RockerBtnDataStruct must be 40 bytes");
static_assert(sizeof(RemoteData) == 40, "RemoteData must be 40 bytes");


// ============================================================
// 4. 自定义按钮 bit mask
//
// idle[4-7]  存 custom_buttons
// idle[8-11] 存 mocap_buttons
// ============================================================

enum CustomButtonBits : uint32_t
{
    CUSTOM_MOVE_TO_DEFAULT_POS = 1u << 0,
    CUSTOM_HOLD_POSITION_MODE  = 1u << 1,
    CUSTOM_CMD_RESET           = 1u << 2,
    CUSTOM_CYCLE_TASK          = 1u << 3,
};

enum MocapButtonBits : uint32_t
{
    MOCAP_EMERGENCY            = 1u << 0,
    MOCAP_MOVE_TO_DEFAULT_POS  = 1u << 1,
    MOCAP_START_POLICY         = 1u << 2,
};


// ============================================================
// 5. CommonGamepad -> wireless_remote[40]
// ============================================================

inline void packCommonGamepadToWirelessRemote(const CommonGamepad& gp, uint8_t wireless_remote[40])
{
    RemoteData remote;
    std::memset(&remote, 0, sizeof(RemoteData));

    // -------------------------------
    // header
    // -------------------------------
    remote.data.head[0] = 0xFE;
    remote.data.head[1] = 0xEF;

    // -------------------------------
    // normal buttons
    // -------------------------------
    remote.data.btn.value = 0;

    remote.data.btn.components.R1     = gp.R1.pressed;
    remote.data.btn.components.L1     = gp.L1.pressed;
    remote.data.btn.components.start  = gp.start.pressed;
    remote.data.btn.components.select = gp.select.pressed;

    remote.data.btn.components.R2     = gp.R2.pressed;
    remote.data.btn.components.L2     = gp.L2.pressed;

    // 这里借用 F1 / F2 放两个最重要的自定义按钮
    remote.data.btn.components.F1     = gp.emergency_btn.pressed;
    remote.data.btn.components.F2     = gp.start_policy_btn.pressed;

    remote.data.btn.components.A      = gp.A.pressed;
    remote.data.btn.components.B      = gp.B.pressed;
    remote.data.btn.components.X      = gp.X.pressed;
    remote.data.btn.components.Y      = gp.Y.pressed;

    remote.data.btn.components.up     = gp.up.pressed;
    remote.data.btn.components.right  = gp.right.pressed;
    remote.data.btn.components.down   = gp.down.pressed;
    remote.data.btn.components.left   = gp.left.pressed;

    // -------------------------------
    // axes
    // -------------------------------
    remote.data.lx = gp.lx;
    remote.data.rx = gp.rx;
    remote.data.ry = gp.ry;
    remote.data.lt = gp.lt;
    remote.data.ly = gp.ly;

    // -------------------------------
    // idle[0-3] 存 rt
    // -------------------------------
    std::memcpy(&remote.data.idle[0], &gp.rt, sizeof(float));

    // -------------------------------
    // idle[4-7] 存其他 custom buttons
    // -------------------------------
    uint32_t custom_buttons = 0;

    if (gp.moveto_default_pos_btn.pressed) custom_buttons |= CUSTOM_MOVE_TO_DEFAULT_POS;
    if (gp.hold_position_mode_btn.pressed) custom_buttons |= CUSTOM_HOLD_POSITION_MODE;
    if (gp.cmd_reset_btn.pressed) custom_buttons |= CUSTOM_CMD_RESET;
    if (gp.cycle_task_btn.pressed) custom_buttons |= CUSTOM_CYCLE_TASK;

    std::memcpy(&remote.data.idle[4], &custom_buttons, sizeof(uint32_t));

    // -------------------------------
    // idle[8-11] 存 mocap buttons
    // -------------------------------
    uint32_t mocap_buttons = 0;

    if (gp.mocap_cmd_emergency_btn.pressed) mocap_buttons |= MOCAP_EMERGENCY;
    if (gp.mocap_cmd_moveto_default_pos_btn.pressed) mocap_buttons |= MOCAP_MOVE_TO_DEFAULT_POS;
    if (gp.mocap_cmd_start_policy_btn.pressed) mocap_buttons |= MOCAP_START_POLICY;
    std::memcpy(&remote.data.idle[8], &mocap_buttons, sizeof(uint32_t));

    // -------------------------------
    // 最后复制到 40 字节数组
    // -------------------------------
    std::memcpy(wireless_remote, remote.buff, 40);
}


// ============================================================
// 6. wireless_remote[40] -> CommonGamepad
// ============================================================

inline bool unpackWirelessRemoteToCommonGamepad(const uint8_t wireless_remote[40], CommonGamepad& gp)
{
    RemoteData remote;
    std::memset(&remote, 0, sizeof(RemoteData));
    std::memcpy(remote.buff, wireless_remote, 40);

    // -------------------------------
    // check header
    // -------------------------------
    if (remote.data.head[0] != 0xFE || remote.data.head[1] != 0xEF)
    {
        return false;
    }

    // -------------------------------
    // normal buttons
    // -------------------------------
    gp.R1.update(remote.data.btn.components.R1);
    gp.L1.update(remote.data.btn.components.L1);
    gp.start.update(remote.data.btn.components.start);
    gp.select.update(remote.data.btn.components.select);

    gp.R2.update(remote.data.btn.components.R2);
    gp.L2.update(remote.data.btn.components.L2);

    gp.emergency_btn.update(remote.data.btn.components.F1);
    gp.start_policy_btn.update(remote.data.btn.components.F2);

    gp.A.update(remote.data.btn.components.A);
    gp.B.update(remote.data.btn.components.B);
    gp.X.update(remote.data.btn.components.X);
    gp.Y.update(remote.data.btn.components.Y);

    gp.up.update(remote.data.btn.components.up);
    gp.right.update(remote.data.btn.components.right);
    gp.down.update(remote.data.btn.components.down);
    gp.left.update(remote.data.btn.components.left);

    // -------------------------------
    // axes
    // -------------------------------
    gp.lx = remote.data.lx;
    gp.rx = remote.data.rx;
    gp.ry = remote.data.ry;
    gp.lt = remote.data.lt;
    gp.ly = remote.data.ly;

    // idle[0-3] 解析 rt
    std::memcpy(&gp.rt, &remote.data.idle[0], sizeof(float));

    // -------------------------------
    // idle[4-7] 解析 custom buttons
    // -------------------------------
    uint32_t custom_buttons = 0;
    std::memcpy(&custom_buttons, &remote.data.idle[4], sizeof(uint32_t));

    gp.moveto_default_pos_btn.update((custom_buttons & CUSTOM_MOVE_TO_DEFAULT_POS) != 0);
    gp.hold_position_mode_btn.update((custom_buttons & CUSTOM_HOLD_POSITION_MODE) != 0);
    gp.cmd_reset_btn.update((custom_buttons & CUSTOM_CMD_RESET) != 0);
    gp.cycle_task_btn.update((custom_buttons & CUSTOM_CYCLE_TASK) != 0);

    // -------------------------------
    // idle[8-11] 解析 mocap buttons
    // -------------------------------
    uint32_t mocap_buttons = 0;
    std::memcpy(&mocap_buttons, &remote.data.idle[8], sizeof(uint32_t));

    gp.mocap_cmd_emergency_btn.update((mocap_buttons & MOCAP_EMERGENCY) != 0);
    gp.mocap_cmd_moveto_default_pos_btn.update((mocap_buttons & MOCAP_MOVE_TO_DEFAULT_POS) != 0);
    gp.mocap_cmd_start_policy_btn.update((mocap_buttons & MOCAP_START_POLICY) != 0);

    // -------------------------------
    // raw 值同步为当前值
    // -------------------------------
    gp.raw_lx = gp.lx;
    gp.raw_ly = gp.ly;
    gp.raw_rx = gp.rx;
    gp.raw_ry = gp.ry;
    gp.raw_lt = gp.lt;
    gp.raw_rt = gp.rt;

    return true;
}

} // namespace common_gamepad_wireless
