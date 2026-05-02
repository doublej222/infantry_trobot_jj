//
// Created by double_J on 2026/4/19.
//

#include <cmath>
#include "controller/pid.h"
#include "ins/ins.h"
#include "motor/dji.h"
#include "utils/msg.h"
#include "utils/os.h"
#include "utils/vofa.h"
/*
 *  全向轮
 *  ^ vy
 *  |       FL              FR
 *  |           / ------ \
 *  |           |        |
 *  |           |        |
 *  |           \ ------ /
 *  |       BL              BR
 *  O-----------------------------> vx
 *
 * C板 在 BL 和 BR 之间
 */
constexpr float SQRT2 = M_SQRT2;
constexpr float fpi = M_PI;
struct chassis_cmd_t {
    float vx;
    float vy;
    float rotate;
    float gimbal_angle;
    uint8_t mode;
    bool rc_status;
};
msg::can_receiver<chassis_cmd_t, E_CAN_2, 0x301> chassis_rx;
using namespace motor;
using namespace controller;
dji motor_1("FL", dji::M3508,dji::param_t{.id = 1,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 14);
dji motor_2("BL", dji::M3508,dji::param_t{.id = 2,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 14);
dji motor_3("BR", dji::M3508,dji::param_t{.id = 3,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 14);
dji motor_4("FR", dji::M3508,dji::param_t{.id = 4,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 14);
pid speed_pid1(450, 0.27, 1, 3000, 16384);
pid speed_pid2(450, 0.27, 1, 3000, 16384);
pid speed_pid3(450, 0.27, 1, 3000, 16384);
pid speed_pid4(450, 0.27, 1, 3000, 16384);

pid yaw_follow_pid(0.01, 0, 0, 3000, 16384);

void chassis_init() {
    motor_1.init(); motor_2.init(); motor_3.init(); motor_4.init();
}
void chassis_update(const float &vx, const float &vy, const float &rotate) {
    float target1 = (rotate + SQRT2 * vx + SQRT2 * vy) * 30 / SQRT2;
    float target2 = (rotate + SQRT2 * vx - SQRT2 * vy) * 30 / SQRT2;
    float target3 = (rotate - SQRT2 * vx - SQRT2 * vy) * 30 / SQRT2;
    float target4 = (rotate - SQRT2 * vx + SQRT2 * vy) * 30 / SQRT2;

    motor_1.update(speed_pid1.update(motor_1.feedback.speed, target1));
    motor_2.update(speed_pid2.update(motor_2.feedback.speed, target2));
    motor_3.update(speed_pid3.update(motor_3.feedback.speed, target3));
    motor_4.update(speed_pid4.update(motor_4.feedback.speed, target4));
}
static float ramp(float target, float current, float step)
{
    float diff = target - current;

    if (diff > step) return current + step;
    if (diff < -step) return current - step;
    return target;
}
float yaw_zero_position = 5.56f;
// float debug;
[[noreturn]] void chassis_task(void *args) {
    chassis_init();
    os::task::sleep(1);

    chassis_rx.init();

    float vx_now = 0, vy_now = 0, rotate_now = 0;

    const auto ins = ins::data();
    float vx_out = 0, vy_out = 0, rotate_out = 0;
    while (true) {
        // gimbal to chassis command
        auto data = chassis_rx();

        if (bsp_time_get_ms() - chassis_rx.timestamp > 100) {
            vx_now = 0; vy_now = 0; rotate_now = 0;
            motor_1.clear(); motor_2.clear(); motor_3.clear(); motor_4.clear();
        }
        float vx = data->vx;
        float vy = data->vy;
        float rotate = data->rotate;
        float chassis_mode = data->mode;
        if (fabsf(vx) < 0.05f) vx = 0;
        if (fabsf(vy) < 0.05f) vy = 0;
        if (fabsf(rotate) < 0.03f) rotate = 0;

        vx_now = ramp(vx, vx_now, 0.12f);
        vy_now = ramp(vy, vy_now, 0.12f);
        rotate_now = ramp(rotate, rotate_now, 0.03f);


        // 待机
        if (chassis_mode == 255) {
            vx_out = 0; vy_out = 0; rotate_now = 0;
        }
        // 右摇杆 上拨 世界坐标系
        if (chassis_mode == 1) {
            float yaw_error = ins->yaw - 0;;
            yaw_error = (yaw_error > fpi) ? (yaw_error - 2 * fpi) : (yaw_error < -fpi) ? yaw_error + 2 * fpi : yaw_error;
            // 旋转矩阵
            float cos_ = std::cos(yaw_error);
            float sin_ = std::sin(yaw_error);

            vx_out = vx_now * cos_ - vy_now * sin_;
            vy_out = vx_now * sin_ + vy_now * cos_;
            rotate_out = rotate_now;
        }
        // 右摇杆回中 跟随云台
        if (chassis_mode == 0) {
            float yaw_angle = data->gimbal_angle;
            float yaw_error = yaw_angle - yaw_zero_position;
            // float yaw_error = ins->yaw - yaw_zero_position;;
            yaw_error = (yaw_error > fpi) ? (yaw_error - 2 * fpi) : (yaw_error < -fpi) ? yaw_error + 2 * fpi : yaw_error;
            // 旋转矩阵
            float cos_ = std::cos(-yaw_error);
            float sin_ = std::sin(-yaw_error);

            vx_out = vx_now * cos_ - vy_now * sin_;
            vy_out = vx_now * sin_ + vy_now * cos_;

            // rotate_follow = yaw_follow_pid.update(yaw_error, 0.0f);
            float rotate_follow = yaw_error * 1.2f;
            if (fabsf(rotate_follow) < 0.1f) rotate_follow = 0;
            rotate_out = rotate_now + rotate_follow;
        }

        if (data->rc_status == false) {
            vx_out = 0; vy_out = 0; rotate_out = 0;
        }
        chassis_update(vx_out, vy_out, rotate_out);

        // vofa::send(E_UART_1, 30, -motor_3.feedback.speed, vx, vy, rotate, ins->yaw, chassis_mode);
        vofa::send(E_UART_1,vx_out, vy_out, rotate_out, data->rc_status, bsp_time_get_ms());
        os::task::sleep(1);
    }
}