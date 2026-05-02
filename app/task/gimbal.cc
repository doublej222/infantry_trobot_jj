//
// Created by double_J on 2026/4/26.
//

#include "utils/os.h"
#include <cmath>
#include "bsp/can.h"
#include "controller/pid.h"
#include "ins/ins.h"
#include "motor/dji.h"
#include "rc/dr16.h"
#include "utils/msg.h"
#include "utils/vofa.h"
constexpr float fpi = M_PI;

using namespace motor;
dji yaw("yaw_motor", dji::GM6020,dji::param_t{.id = 1,.port = E_CAN_1,.mode = dji::CURRENT}, -1);
dji pitch("pitch_motor", dji::GM6020, dji::param_t{.id = 2,.port = E_CAN_1, .mode = dji::CURRENT}, -1);
controller::pid yaw_speed_pid(760, 0.65, 0, 4000, 16384);
controller::pid yaw_pos_pid(30, 0, 3, 0, 20);
controller::pid pitch_speed_pid(760, 0.65, 0, 4000, 16384);
controller::pid pitch_pos_pid(60, 0, 3, 0, 20);

// 电机控制量
float delta_yaw_target, yaw_target, yaw_speed, yaw_output;

[[noreturn]] void gimbal_task(void *args) {
    yaw.init(); pitch.init();
    auto ins = ins::data();
    auto rc = rc::dr16::data();
    os::task::sleep(1);
    delta_yaw_target = ins::data()->yaw_total_angle;
    float measure_speed = 0;
    while (true) {
        // 遥控器 dr16
        if (rc->s_r == 0) {
            delta_yaw_target = static_cast<float>(rc::dr16::data()->rc_r[0]) * 0.0007f;
            yaw_target = -delta_yaw_target + ins->yaw_total_angle;
        }else {
            yaw_speed = 0, yaw_output = 0;
        }

        // 一阶低通滤波：alpha越大越灵敏，滤波效果越小
        constexpr float alpha = 0.3;
        measure_speed = (1 - alpha) * measure_speed + alpha * yaw.feedback.speed;

        // control
        // ===============================yaw================================
        yaw_speed = yaw_pos_pid(ins->yaw_total_angle, yaw_target);
        yaw_output = yaw_speed_pid(measure_speed, yaw_speed);
        yaw.update(yaw_output);
        // ==============================pitch===============================
        pitch.update(pitch_speed_pid.update(pitch.feedback.speed, pitch_pos_pid.update(pitch.feedback.angle, 5.6f)));

        vofa::send(E_UART_6, yaw.feedback.speed);
        os::task::sleep(1);
    }
}
