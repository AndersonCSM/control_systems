#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#define PI_F     3.14159265358979f
#define DEG2RAD  (PI_F / 180.0f)

/* Frequência de corte do filtro passa-baixas da derivada (Hz).
 * Suaviza a quantização de 0.15° do encoder em 1 kHz sem atrasar a fase do LGR.
 */
#define DERIV_FILT_HZ  15.0f

typedef struct {
    float Kp, Ki, Kd;      /* ganhos calculados a partir de K e z        */
    float dt;              /* período de amostragem (s)                  */
    float limite_torque;   /* saturação máxima de torque (N.m)           */
    float integral;        /* acumulador da ação integral                */
    float erro_anterior;   /* erro da amostra anterior (rad)             */
    float deriv_filtrada;  /* derivada filtrada passa-baixas (rad/s)     */
    float alpha_d;         /* coeficiente do filtro IIR da derivada      */
} PID_t;

/* Inicialização a partir dos parâmetros K e z do LGR:
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
