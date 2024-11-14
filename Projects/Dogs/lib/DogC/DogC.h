#ifndef DOGC_H
#define DOGC_H

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>

#define deg2rad(x) (x * PI / 180.0)
#define rad2deg(x) (x * 180.0 / PI)

#define LH_A 9
#define LH_B 8
#define RH_A 11
#define RH_B 10
#define LF_A 2
#define LF_B 4
#define RF_A 0
#define RF_B 1
// 大腿舵机逆运动学得到的角度与实际舵机角度差九十度；小腿舵机逆运动学得到的角度与实际角度相同
// 在输出角度时要加上对应的偏差值
#define RH_A_BIAS 100.0
#define RH_B_BIAS 0.0
#define LH_A_BIAS 80
#define LH_B_BIAS 10.0

#define RF_A_BIAS 95
#define RF_B_BIAS 0.0
#define LF_A_BIAS 100
#define LF_B_BIAS 10.0

#define MAX_ANGLE 180.0
#define MIN_ANGLE 0.0
#define MAX_PULSE 2.5
#define MIN_PULSE 0.5

#define SERVOMIN 150.0  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600.0  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600.0     // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400.0    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50.0 // Analog servos run at ~50 Hz updates
#define RESOLUTION 8.0

// 狗腿长度参数,单位mm
#define L1 85.0 // 大腿长度---两旋转轴间距
#define L2 70.0 // 小腿长度---小腿旋转轴与足端间距
#define L3 15.0 // 小腿控制舵机臂长度
#define L4 20.0 // 拉杆端和小腿旋转轴间距
#define L5 65.0 // 足端到小腿控制舵机臂长度

#define X0 70.0
#define Y0 26.0
#define Z0 7.0

// 步态参数
#define Ts 1
#define fai 0.5

#define init_x 60.0
#define init_y 0.0
#define ges_x_1 init_x
#define ges_y_1 init_y
#define ges_x_2 init_x
#define ges_y_2 init_y
#define ges_x_3 init_x
#define ges_y_3 init_y
#define ges_x_4 init_x
#define ges_y_4 init_y

/*!
 *  @brief  输入旋转角度，转动髋关节四个舵机
 */
void hip_move_angle(Adafruit_PWMServoDriver pwm, int servos[8], float angle);

/*!
 *  @brief  输入旋转角度，转动膝关节四个舵机
 */
void knee_move_angle(Adafruit_PWMServoDriver pwm, int servos[8], float angle);

/*!
 *  @brief  输入需要转的角度，转化为脉冲宽度
 */
int angle2pulse(float angle, float min_angle, float max_angle, float min_pulse, float max_pulse);
/*!
 *  @brief  初始化舵机到90度位置
 */
void angle_init(Adafruit_PWMServoDriver pwm, int *servos, int size);
/*!
 *  @brief  保持舵机角度到指定角度不变
 */
void angle_keep(Adafruit_PWMServoDriver pwm, float angle, int *servos, int size);
/*!
 *  @brief  机械狗足端坐标反解,反解得到的角度为相对于狗身坐标系的角度
 */
void dog_inverse(float fx, float fy, float *alpha, float *beta);
/*!
 *  @brief  机械狗四个足端坐标得到八个舵机转动角度
 */
void dog_inverse_legs(float RHX, float RHY, float LHX, float LHY, float RFX, float RFY, float LFX, float LFY, float angles[8]);
/*!
 *  @brief  传入四个足端位置坐标，完成机械狗的动作
 */
void dog_move_legs(Adafruit_PWMServoDriver pwm, int servos[8], double x_pos[4], double y_pos[4]);
/*!
 *  @brief  t:生成时间，ys：起始位置，yf：终止位置，h：抬腿高度，r1、r2、r3、r4：控制方向，-1往后，1往前，0不动，y_pos：四个足端的y坐标，x_pos：四个足端的x坐标
 */
void trot_plan(double t, double ys, double yf, double h, double r1, double r2, double r3, double r4, double y_pos[4], double x_pos[4]);
#endif