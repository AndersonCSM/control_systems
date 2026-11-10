#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float dt, float limite_torque)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->limite_torque = limite_torque;
    PID_Reset(pid);
}

/* K > 0, z > 0  →  Kp, Ki, Kd todos positivos */
void PID_InitKz(PID_t *pid, float K, float z,
                float dt, float limite_torque)
{
    PID_Init(pid,
             2.0f * K * z,   /* Kp */
             K * z * z,     /* Ki */
             K,             /* Kd */
             dt, limite_torque);
}

void PID_Reset(PID_t *pid)
{
    pid->integral      = 0.0f;
    pid->erro_anterior = 0.0f;
}

float PID_Calcular(PID_t *pid, float setpoint_graus,
                   float angulo_encoder_graus)
{
    /* 1) Erro em rad (mesma unidade da FT) */
    float e = (setpoint_graus - angulo_encoder_graus) * DEG2RAD;

    /* 2) Termos P e D puros (sem filtros extras na planta) */
    float P = pid->Kp * e;
    float D = pid->Kd * (e - pid->erro_anterior) / pid->dt;

    /* 3) Termo I com limitação de torque */
    float i_novo = pid->integral + e * pid->dt;
    float I      = pid->Ki * i_novo;

    float u = P + I + D;

    if (u > pid->limite_torque) {
        u = pid->limite_torque;
        if (e < 0.0f) pid->integral = i_novo;
    } else if (u < -pid->limite_torque) {
        u = -pid->limite_torque;
        if (e > 0.0f) pid->integral = i_novo;
    } else {
        pid->integral = i_novo;
    }

    pid->erro_anterior = e;
    return u;
}
