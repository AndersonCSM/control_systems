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
    /* Coeficiente do filtro passa-baixas de 1ª ordem na derivada:
     * alpha_d = 1 - exp(-2*pi*fc*dt)
     */
    pid->alpha_d = 1.0f - expf(-2.0f * PI_F * DERIV_FILT_HZ * dt);
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
    pid->integral       = 0.0f;
    pid->erro_anterior  = 0.0f;
    pid->deriv_filtrada = 0.0f;
}

float PID_Calcular(PID_t *pid, float setpoint_graus,
                   float angulo_encoder_graus)
{
    /* 1) Erro em graus e em radianos */
    float erro_graus = setpoint_graus - angulo_encoder_graus;
    float e = erro_graus * DEG2RAD;

    /* 2) Termo P e Termo D com filtro passa-baixas de 1ª ordem (15 Hz) */
    float P = pid->Kp * e;

    float deriv_bruta = (e - pid->erro_anterior) / pid->dt;
    pid->deriv_filtrada += pid->alpha_d * (deriv_bruta - pid->deriv_filtrada);
    float D = pid->Kd * pid->deriv_filtrada;

    /* 3) Termo I com Anti-Windup Inteligente e Zona Morta para Eliminar Trepidação:
     *    - |erro| > 5.0° (subida manual / longe do topo): zera o integrador para
     *      evitar que o erro acumulado na subida arremesse a haste ao chegar em 180°.
     *    - |erro| <= 0.30° (haste estável no topo, ~2 pulsos do encoder):
     *      CONGELA o integrador no valor atual, desligando novas somas para
     *      eliminar a caça ao ruído de quantização (elimina trepidação contínua).
     *    - 0.30° < |erro| <= 5.0° (região de distúrbio):
     *      integração PURA (sem leak_i), acumulando a força necessária para
     *      vencer atritos, toques externos ou desnível da mesa.
     */
    float abs_erro_graus = fabsf(erro_graus);
    float i_novo = pid->integral;

    if (abs_erro_graus > 5.0f) {
        i_novo = 0.0f;                        /* Anti-windup de subida manual */
    } else if (abs_erro_graus > 0.30f) {
        i_novo = pid->integral + e * pid->dt; /* Integração pura para rejeição de distúrbios */
    }
    /* Se abs_erro_graus <= 0.30f: mantém i_novo = pid->integral (estável no topo) */

    float I = pid->Ki * i_novo;
    float u = P + I + D;

    /* 4) Anti-windup por saturação física do motor (+- 0.14 N.m) */
    if (u > pid->limite_torque) {
        u = pid->limite_torque;
        if (e < 0.0f) pid->integral = i_novo; /* Só aceita se o erro estiver descarregando o teto */
    } else if (u < -pid->limite_torque) {
        u = -pid->limite_torque;
        if (e > 0.0f) pid->integral = i_novo; /* Só aceita se o erro estiver descarregando o piso */
    } else {
        pid->integral = i_novo;
    }

    pid->erro_anterior = e;
    return u;
}
