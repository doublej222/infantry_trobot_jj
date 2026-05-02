//
// Created by double_J on 2026/4/26.
//
#include "motor/dji.h"
#include "rc/dr16.h"
#include "utils/msg.h"
#include "utils/vofa.h"
extern motor::dji yaw;
struct chassis_cmd_t {
    float vx;
    float vy;
    float rotate;
    float gimbal_angle;
    uint8_t mode;
    bool rc_status;
};
msg::can_sender<chassis_cmd_t, E_CAN_2, 0x301> chassis_tx(5);

[[noreturn]]void msg_task(void *args) {
    chassis_tx.init();
    os::task::sleep(1);

    while (true) {
        const auto rc = rc::dr16::data();

        auto data = chassis_tx();
        // 左摇杆 chassis
        data->vx = static_cast<float>(rc->rc_l[1]) / 660.0f;
        data->vy = static_cast<float>(rc->rc_l[0]) / 660.0f;
        // 右摇杆 gimbal
        data->rotate = static_cast<float>(rc->reserved) / 660.0f;

        data->gimbal_angle = yaw.feedback.angle;
        // 开关
        data->mode = static_cast<uint8_t>(rc->s_r);

        // 遥控掉电保护
        if (bsp_time_get_ms() - rc->timestamp > 100) data->rc_status = false;
        else data->rc_status = true;

        chassis_tx.send();
        os::task::sleep(1);
    }
}