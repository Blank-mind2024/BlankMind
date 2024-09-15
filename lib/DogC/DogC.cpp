#include "DogC.h"

int angle2pulse(float angle, float min_angle, float max_angle, float min_pulse, float max_pulse)
{
    angle = (angle - min_angle) / (max_angle - min_angle) * (max_pulse - min_pulse) + min_pulse;
    int pulse = angle / 20 * 4096;
    return pulse;
}

void hip_move_angle(Adafruit_PWMServoDriver pwm, int servos[8], float angle)
{
    pwm.setPWM(servos[0], 0, angle2pulse(RH_A_BIAS + angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[2], 0, angle2pulse(LH_A_BIAS - angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[4], 0, angle2pulse(RF_A_BIAS + angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[6], 0, angle2pulse(LF_A_BIAS - angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    // delay(300);
}
void knee_move_angle(Adafruit_PWMServoDriver pwm, int servos[8], float angle)
{
    pwm.setPWM(servos[1], 0, angle2pulse(angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[3], 0, angle2pulse(180 - angle - LH_B_BIAS, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[5], 0, angle2pulse(angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[7], 0, angle2pulse(180 - angle - LF_B_BIAS, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    // delay(300);
}

void angle_init(Adafruit_PWMServoDriver pwm, int *servos, int size)
{
    for (int *i = servos; i < servos + size; i++)
    {
        pwm.setPWM(*i, 0, angle2pulse(90, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    }
    delay(3000);
}

void angle_keep(Adafruit_PWMServoDriver pwm, float angle, int *servos, int size)
{
    for (int *i = servos; i < servos + size; i++)
    {
        int pulse = angle2pulse(angle, MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE);
        pwm.setPWM(*i, 0, pulse);
    }
    delay(1000);
}
void dog_inverse(float fx, float fy, float *alpha, float *beta)
{
    // 足端和大腿旋转轴与x轴夹角,正弦函数输入输出均为弧度
    float phi = acos((pow(fx, 2) + pow(fy, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * sqrt(pow(fx, 2) + pow(fy, 2)))); // 弧度
    *alpha = degrees(atan2(fy, fx) - phi);                                                                            // 角度

    // 逆解存在条件：1.98<beta2<5.14
    float beta2 = PI - acos((pow(fx, 2) + pow(fy, 2) - pow(L1, 2) - pow(L2, 2)) / (-2 * L1 * L2)); // 弧度
    if (beta2 < 2)
    {
        beta2 = 5.14 - 1.98 - beta2;
    }
    float C = (pow(L4, 2) + pow(L3, 2) + pow(X0, 2) + pow(Y0, 2) + pow(Z0, 2) + 2 * X0 * L4 * cos(beta2) + 2 * Y0 * L4 * sin(beta2) - pow(L5, 2)) / (-2 * L3);
    float A = Z0;
    float B = X0 + L4 * cos(beta2);
    float phi2 = atan2(B, A); // 弧度
    beta2 = asin(C / (sqrt(pow(A, 2) + pow(B, 2)))) - phi2;
    *beta = degrees(-beta2);
}

void dog_inverse_legs(float RHX, float RHY, float LHX, float LHY, float RFX, float RFY, float LFX, float LFY, float angles[8])
{
    float alpha;
    float beta;
    dog_inverse(RHX, RHY, &alpha, &beta);
    angles[0] = RH_A_BIAS + alpha;
    // angles[1] = beta;
    angles[1] = 180 - beta;
    dog_inverse(LHX, LHY, &alpha, &beta);
    angles[2] = LH_A_BIAS - alpha;
    // angles[3] = 180 - beta - LH_B_BIAS;
    angles[3] = beta + LH_B_BIAS;
    dog_inverse(RFX, RFY, &alpha, &beta);
    angles[4] = RF_A_BIAS + alpha;
    // angles[5] = beta;
    angles[5] = 180 - beta;
    dog_inverse(LFX, LFY, &alpha, &beta);
    angles[6] = LF_A_BIAS - alpha;
    // angles[7] = 180 - beta - LF_B_BIAS;
    angles[7] = beta + LF_B_BIAS;
}
void dog_move_legs(Adafruit_PWMServoDriver pwm, int servos[8], double x_pos[4], double y_pos[4])
{
    float angles[8];
    dog_inverse_legs(x_pos[0], y_pos[0], x_pos[1], y_pos[1], x_pos[2], y_pos[2], x_pos[3], y_pos[3], angles);
    pwm.setPWM(servos[0], 0, angle2pulse(angles[0], MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[1], 0, angle2pulse(angles[1], MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[2], 0, angle2pulse(angles[2], MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[3], 0, angle2pulse(angles[3], MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[4], 0, angle2pulse(angles[4], MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[5], 0, angle2pulse(angles[5], MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[6], 0, angle2pulse(angles[6], MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    pwm.setPWM(servos[7], 0, angle2pulse(angles[7], MIN_ANGLE, MAX_ANGLE, MIN_PULSE, MAX_PULSE));
    delay(20);
}
void trot_plan(double t, double ys, double yf, double h, double r1, double r2, double r3, double r4, double y_pos[4], double x_pos[4])
{
    float sigma = 0;
    float x_exp = 0;
    float y_pos_b = 0;
    float y_pos_z = 0;
    if (t <= Ts * fai)
    {
        /* 前半周期0,3是摆动相；1,2是支撑相 */
        sigma = 2 * PI * t / (fai * Ts);
        x_exp = h * (1 - cos(sigma)) / 2;
        y_pos_b = (yf - ys) * (sigma - sin(sigma)) / (2 * PI) + ys; // 摆动相
        y_pos_z = (ys - yf) * (sigma + sin(sigma)) / (2 * PI) + yf; // 支撑相
        y_pos[0] = y_pos_b * r1;
        y_pos[1] = y_pos_z * r2;
        y_pos[2] = y_pos_z * r3;
        y_pos[3] = y_pos_b * r4;

        x_pos[0] = ges_x_1 - x_exp;
        x_pos[1] = ges_x_2;
        x_pos[2] = ges_x_3;
        x_pos[3] = ges_x_4 - x_exp;
    }
    else
    {
        /* 后半周期1,2是摆动相;0,3是支撑相 */
        sigma = 2 * PI * (t - fai * Ts) / (fai * Ts);
        x_exp = h * (1 - cos(sigma)) / 2;
        y_pos_b = (yf - ys) * (sigma - sin(sigma)) / (2 * PI) + ys; // 摆动相
        y_pos_z = (ys - yf) * (sigma + sin(sigma)) / (2 * PI) + yf; // 支撑相
        y_pos[0] = y_pos_z * r1;
        y_pos[1] = y_pos_b * r2;
        y_pos[2] = y_pos_b * r3;
        y_pos[3] = y_pos_z * r4;

        x_pos[0] = ges_x_1;
        x_pos[1] = ges_x_2 - x_exp;
        x_pos[2] = ges_x_3 - x_exp;
        x_pos[3] = ges_x_4;
    }
}