#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#define PI_F     3.14159265358979f
#define DEG2RAD  (PI_F / 180.0f)

typedef struct {
    float Kp, Ki, Kd;      /* ganhos calculados a partir de K e z      */
    float dt;              /* período de amostragem (s)                */
    float limite_torque;   /* saturação (N.m)                          */
    float integral;        /* acumulador do integrador                 */
    float erro_anterior;   /* erro anterior em rad                     */
} PID_t;

/* Inicialização PRINCIPAL: a partir do K e z do LGR.
 *   Kp = 2*K*z     Ki = K*z^2     Kd = K
 */
void PID_InitKz(PID_t *pid, float K, float z,
                float dt, float limite_torque);

/* Inicialização alternativa com ganhos diretos */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float dt, float limite_torque);

void  PID_Reset(PID_t *pid);

float PID_Calcular(PID_t *pid, float setpoint_graus,
                   float angulo_encoder_graus);

#ifdef __cplusplus
}
#endif

#endif
