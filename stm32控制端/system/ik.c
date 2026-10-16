#include "stm32f10x.h"		//逆运动学算法
#include <math.h>

#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.0174532925f


static float last_j2 = 90.0f;
static float last_j3 = 90.0f;
static float last_j4 = 0.0f;

void ik_get_angle(float cx, float cy, float angle, float *j_out)
{
    float a1, a2, a3, a4;
    float P;
    float X, Y, Z;
    float len, high;
    float L, H;
    float Cosj2, Cosj3;
    float Sinj2, Sinj3;
    float K1, K2;
    float j1;
    float j2, j3, j4;
    float j_all;


    float best_j2 = last_j2;
    float best_j3 = last_j3;
    float best_j4 = last_j4;

    float best_all_j2 = last_j2, best_all_j3 = last_j3, best_all_j4 = last_j4;
    float best_gt30_j2 = last_j2, best_gt30_j3 = last_j3, best_gt30_j4 = last_j4;
    float best_all_score = 1e9f, best_gt30_score = 1e9f;
    int found_all = 0;
    int found_gt30 = 0;

    X = cx;				//物块所处的x轴坐标
    Y = cy;				//物块所处的y轴坐标
    Z = 4;				//z坐标，这里我固定为4，你也可以将识别到的z坐标输入
    
	P = 10.0f; 			//底座偏移角度
    a1 = 80.0f;			//底座高
    a2 = 104.0f;		//大臂长
    a3 = 96.31f;		//中臂长
    a4 = 163.3f;		//小臂长

    if (angle > 90) {
        angle -= 90;
    }

    j1 = atan2f(Y + P, X) * RAD_TO_DEG;
    len = sqrtf(X * X + (Y + P) * (Y + P));
    high = Z;

    for (float i = 0.0f; i <= 180.0f; i += 0.2f) {
        j_all = i;

        L = len - a4 * sinf(j_all * DEG_TO_RAD);
        H = high - a4 * cosf(j_all * DEG_TO_RAD) - a1;

        float r2 = L * L + H * H;
        float max_r = a2 + a3;
        float min_r = fabsf(a2 - a3);

        if (r2 > max_r * max_r)
            continue;
        if (r2 < min_r * min_r)
            continue;

        Cosj3 = (L * L + H * H - a2 * a2 - a3 * a3) / (2.0f * a2 * a3);
        if (Cosj3 > 1.0f) Cosj3 = 1.0f;
        if (Cosj3 < -1.0f) Cosj3 = -1.0f;
        Sinj3 = sqrtf(1.0f - Cosj3 * Cosj3);

        j3 = atan2f(Sinj3, Cosj3) * RAD_TO_DEG;

        K1 = a2 + a3 * cosf(j3 * DEG_TO_RAD);
        K2 = a3 * sinf(j3 * DEG_TO_RAD);

        Cosj2 = (K2 * L + K1 * H) / (K1 * K1 + K2 * K2);
        if (Cosj2 > 1.0f) Cosj2 = 1.0f;
        if (Cosj2 < -1.0f) Cosj2 = -1.0f;
        Sinj2 = sqrtf(1.0f - Cosj2 * Cosj2);

        j2 = atan2f(Sinj2, Cosj2) * RAD_TO_DEG;
        j4 = j_all - j2 - j3;

 
        if (isnan(j2) || isnan(j3) || isnan(j4))
            continue;
        if (j2 < 0 || j2 > 180)
            continue;
        if (j3 < 0 || j3 > 180)
            continue;
        if (j4 < -90 || j4 > 90)
            continue;

        found_all = 1;
        float score = fabsf(j2 - last_j2) + fabsf(j3 - last_j3) + fabsf(j4 - last_j4);

        if (score < best_all_score) {
            best_all_score = score;
            best_all_j2 = j2;
            best_all_j3 = j3;
            best_all_j4 = j4;
        }

        if (j4 > 30.0f) {
            found_gt30 = 1;
            if (score < best_gt30_score) {
                best_gt30_score = score;
                best_gt30_j2 = j2;
                best_gt30_j3 = j3;
                best_gt30_j4 = j4;
            }
        }
    }

    if (found_gt30) {
        best_j2 = best_gt30_j2;
        best_j3 = best_gt30_j3;
        best_j4 = best_gt30_j4;
    } else if (found_all) {
        best_j2 = best_all_j2;
        best_j3 = best_all_j3;
        best_j4 = best_all_j4;
    } else {

        best_j2 = last_j2;
        best_j3 = last_j3;
        best_j4 = last_j4;
    }

    last_j2 = best_j2;
    last_j3 = best_j3;
    last_j4 = best_j4;


    float servo1 = 173.0f - fabsf(j1);		//由于舵机安装问题我这里加了偏转，如不需要可删除
    float servo2 = 117.0f - best_j2;		//由于舵机安装问题我这里加了偏转，如不需要可删除
    float servo3 = best_j3 + 1.0f;			//由于舵机安装问题我这里加了偏转，如不需要可删除
    float servo4 = best_j4 + 1.0f;			//由于舵机安装问题我这里加了偏转，如不需要可删除
    float servo5;

    if (fabsf(best_j4) > 45.0f) {
        float delta = angle - j1;
        while (delta >= 180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        servo5 = 180.0f + delta;
    } else {
        servo5 = 180.0f;
    }


    if (servo1 < 0.0f) servo1 = 0.0f;
    if (servo1 > 180.0f) servo1 = 180.0f;
    if (servo2 < 0.0f) servo2 = 0.0f;
    if (servo2 > 180.0f) servo2 = 180.0f;
    if (servo3 < 0.0f) servo3 = 0.0f;
    if (servo3 > 180.0f) servo3 = 180.0f;
    if (servo4 < 0.0f) servo4 = 0.0f;
    if (servo4 > 180.0f) servo4 = 180.0f;
    if (servo5 < 0.0f) servo5 = 0.0f;
    if (servo5 > 180.0f) servo5 = 180.0f;

    j_out[0] = servo1;
    j_out[1] = servo2;
    j_out[2] = servo3;
    j_out[3] = servo4;
    j_out[4] = servo5;
}


