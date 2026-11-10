#include <Arduino.h>
#include "pid.h"
#include <math.h>
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

/* ============================================================
 * PARÂMETROS DO SISTEMA (Calculados via LGR para m=8.5g, l=29cm)
 * Planta pura G(s) = (1 / m*l^2) / (s^2 - g/l) e controlador PID puro
 * ============================================================ */
#define K_LGR            0.015601f // Ganho K = Kd do LGR (positivo)
#define Z_LGR            5.9888f   // Zero duplo s = -z do LGR (rad/s)
#define DT               0.001f    // 1 ms → 1 kHz
#define TORQUE_MAX_NM    0.14f     // Torque máximo via A4988 (bipolar)
#define SETPOINT_GRAUS   180.0f    // Topo = 180° (haste para baixo = 0°)
#define ZONA_ATIVA_GRAUS 30.0f     // Atua quando a haste está no topo (180° ± 30°)

/* Parâmetros físicos da FT para conversão Torque -> Aceleração -> Freq STEP */
#define M_MASSA          0.0085f   // 8.5 g -> kg
#define L_COMP           0.29f     // 29 cm -> m
#define J_INERCIA        (M_MASSA * L_COMP * L_COMP) // 0.00071485 kg.m^2
#define PASSOS_POR_VOLTA 3200.0f   // 200 passos * 16 microsteps (A4988)

/* ============================================================
 * PINAGEM ESP32 (DevKit 30 pinos)
 * ============================================================ */
#define PIN_ENC_A     32   // fio VERDE do encoder  (pull-up p/ 3V3!)
#define PIN_ENC_B     33   // fio BRANCO do encoder (pull-up p/ 3V3!)
#define PIN_STEP      25   // driver STEP
#define PIN_DIR       26   // driver DIR
#define PIN_EN        27   // driver ENABLE (ativo em nível baixo)

#define PPR            600       // Pulsos por volta do encoder
#define CPR            (PPR * 4) // 2400 contagens por volta em quadratura x4
#define STEP_ISR_HZ    20000UL   // Timer de 20 kHz para leitura limpa do encoder + pulsos STEP
#define FREQ_MAX_HZ    4000.0f   // Frequência máxima de STEP (Hz)

/* ============================================================
 * VARIÁVEIS GLOBAIS ORGANIZADAS POR SUBSISTEMA
 * ============================================================ */

/* 1) Microcontrolador, Timer de Tempo Real (20 kHz -> 1 kHz) e PID */
hw_timer_t *     timer         = NULL;  // Ponteiro do Hardware Timer do ESP32 (dispara a cada 50 us)
volatile bool    flag_1khz     = false; // Bandeira que libera o cálculo do PID a exatos 1 kHz (1 ms)
static uint8_t   s_div_1khz    = 0;     // Contador divisor de frequência (conta 20 ticks de 50 us = 1 ms)
static PID_t     pid;                   // Estrutura principal do controlador PID

/* 2) Encoder Incremental em Quadratura (Haste do Pêndulo) */
volatile long    encoder_count = 0;     // Contagem acumulada de pulsos do encoder (2400 pulsos/volta)
volatile uint8_t enc_state     = 0;     // Histórico de 4 bits dos canais A/B ([A_ant B_ant A_atual B_atual])

/* Tabela de transição de estados de quadratura (rejeita bounce/ruído em pino único) */
static const int8_t DRAM_ATTR QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/* Leitura ultra-rápida dos GPIOs 32 e 33 direto do registrador de hardware in1 */
static inline uint8_t IRAM_ATTR ler_ab_fast(void) {
    uint32_t in1 = GPIO.in1.val;
    uint8_t a = (in1 >> (PIN_ENC_A - 32)) & 1U;
    uint8_t b = (in1 >> (PIN_ENC_B - 32)) & 1U;
    return (a << 1) | b;
}

/* 3) Motor de Passo (Gerador de Pulsos DDS a 20 kHz) */
#define STEP_MASK (1UL << PIN_STEP)     // Máscara de bit do pino STEP (GPIO 25) p/ escrita rápida em registrador
#define DIR_MASK  (1UL << PIN_DIR)      // Máscara de bit do pino DIR  (GPIO 26) p/ escrita rápida em registrador

volatile int32_t g_rate_half   = 0;     // Passo de incremento do DDS proporcional à frequência (Hz) comandada
static uint32_t  s_phase       = 0;     // Acumulador de fase de 32 bits (quando transborda, emite 1 pulso STEP)
static bool      s_pulse_high  = false; // Indica se o pino STEP está em HIGH (para baixá-lo após 50 us)
static bool      s_pending     = false; // Guarda pedido de pulso STEP caso o transbordo ocorra durante a descida do pino
static int8_t    s_dir_cur     = 1;     // Sentido atual do motor (+1 ou -1) para detectar inversão do pino DIR
static float     vel_cmd_rad_s = 0.0f;  // Velocidade angular do motor (rad/s) obtida integrando a aceleração (tau/J)

/* ISR de 20 kHz: lê o encoder sem perder pulsos, gera o trem de pulsos STEP
 * para o A4988 e marca a flag de 1 kHz (1 ms) para o PID.
 */
void IRAM_ATTR onTimer20kHz(void) {
    // 1) Leitura do encoder em quadratura x4
    uint8_t ab = ler_ab_fast();
    enc_state = ((enc_state << 2) | ab) & 0x0F;
    encoder_count += QUAD_TABLE[enc_state];

    // 2) Divisor 20 kHz -> 1 kHz (1 ms) para o loop do PID
    if (++s_div_1khz >= 20) {
        s_div_1khz = 0;
        flag_1khz = true;
    }

    // 3) Geração dos pulsos STEP no pino 25
    bool just_cleared = false;
    if (s_pulse_high) {
        REG_WRITE(GPIO_OUT_W1TC_REG, STEP_MASK);
        s_pulse_high = false;
        just_cleared = true;
    }

    int32_t r    = g_rate_half;
    int8_t  dir  = (r >= 0) ? 1 : -1;
    uint32_t inc = (uint32_t)(r >= 0 ? r : -r) << 1;

    if (dir != s_dir_cur) {
        REG_WRITE((dir > 0) ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, DIR_MASK);
        s_dir_cur = dir;
        s_pending = false;
        return;
    }

    uint32_t old = s_phase;
    s_phase += inc;
    if (s_phase < old) s_pending = true;

    if (s_pending && !just_cleared) {
        REG_WRITE(GPIO_OUT_W1TS_REG, STEP_MASK);
        s_pulse_high = true;
        s_pending    = false;
    }
}

/* ============================================================
 * FUNÇÕES DE HARDWARE E CONVERSÃO FÍSICA (TORQUE -> FREQ STEP)
 * ============================================================ */
float encoder_ler_graus(void) {
    long count = encoder_count;
    long mod = count % CPR;
    if (mod < 0) mod += CPR;
    return (float)mod * 360.0f / (float)CPR;
}

static void aplicar_torque_motor(float tau_nm) {
    if (tau_nm >  TORQUE_MAX_NM) tau_nm =  TORQUE_MAX_NM;
    if (tau_nm < -TORQUE_MAX_NM) tau_nm = -TORQUE_MAX_NM;

    // Equação da planta: alpha = tau / J  ->  vel_cmd += alpha * dt
    float alpha_cmd = tau_nm / J_INERCIA;
    vel_cmd_rad_s += alpha_cmd * DT;

    // Velocidade angular (rad/s) -> Frequência de passos STEP (Hz)
    float freq_signed = vel_cmd_rad_s * (PASSOS_POR_VOLTA / (2.0f * PI_F));

    if (freq_signed >  FREQ_MAX_HZ) {
        freq_signed =  FREQ_MAX_HZ;
        vel_cmd_rad_s = freq_signed * (2.0f * PI_F / PASSOS_POR_VOLTA);
    } else if (freq_signed < -FREQ_MAX_HZ) {
        freq_signed = -FREQ_MAX_HZ;
        vel_cmd_rad_s = freq_signed * (2.0f * PI_F / PASSOS_POR_VOLTA);
    }

    g_rate_half = (int32_t)(freq_signed * (2147483648.0f / (float)STEP_ISR_HZ));
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("==================================================");
    Serial.println("BOOT OK - Controle 2 SIMPLES (PID Puro + FT Pura)");
    Serial.printf("PINOS -> STEP:%d DIR:%d EN:%d ENC_A:%d ENC_B:%d\n",
                  PIN_STEP, PIN_DIR, PIN_EN, PIN_ENC_A, PIN_ENC_B);
    Serial.println("Comando no Monitor Serial (115200 baud):");
    Serial.println("  'c' -> Zerar angulo de referencia com haste embaixo (0.00°)");
    Serial.println("==================================================");

    // 1. Inicializa o controlador PID com os parâmetros do LGR
    PID_InitKz(&pid, K_LGR, Z_LGR, DT, TORQUE_MAX_NM);

    // 2. Configuração dos pinos do Driver A4988
    pinMode(PIN_STEP, OUTPUT);
    digitalWrite(PIN_STEP, LOW);
    pinMode(PIN_DIR, OUTPUT);
    digitalWrite(PIN_DIR, HIGH);
    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, LOW); // Habilita o driver A4988

    // 3. Configuração do Encoder
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    enc_state = ler_ab_fast();

    // 4. Configuração do Timer de 20 kHz
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer20kHz);
    timerAlarm(timer, 1000000UL / STEP_ISR_HZ, true, 0);
#else
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer20kHz, true);
    timerAlarmWrite(timer, 1000000UL / STEP_ISR_HZ, true);
    timerAlarmEnable(timer);
#endif
}

void loop() {
    static bool controle_ativo = false;

    // Interface Serial única: 'c' para zerar o ângulo de referência (0.00°)
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == 'c' || ch == 'C') {
            encoder_count = 0;
            Serial.println("\n>>> [MANUAL] Angulo de referencia zerado na posicao atual (0.00 graus)!\n");
        }
    }

    // Malha de controle discreta a exatos 1 kHz (dt = 1 ms)
    if (flag_1khz) {
        flag_1khz = false;

        // 1) Leitura do ângulo do encoder (0° embaixo, 180° no topo)
        float ang = encoder_ler_graus();
        float tau = 0.0f;

        // 2) Atua na região linearizada do topo (180° ± 30°)
        if (fabsf(SETPOINT_GRAUS - ang) <= ZONA_ATIVA_GRAUS) {
            if (!controle_ativo) {
                PID_Reset(&pid);
                pid.erro_anterior = (SETPOINT_GRAUS - ang) * DEG2RAD;
                vel_cmd_rad_s     = 0.0f;
                controle_ativo    = true;
            }

            // 3) Controlador PID puro projetado via LGR (sem termos extras de braço)
            tau = -PID_Calcular(&pid, SETPOINT_GRAUS, ang);

            // 4) Aplica o torque na planta (integra alpha = tau/J -> freq STEP)
            aplicar_torque_motor(tau);
        } else {
            if (controle_ativo) {
                controle_ativo = false;
                vel_cmd_rad_s  = 0.0f;
                g_rate_half    = 0;
                PID_Reset(&pid);
            }
        }

        // Log a cada 100 ms
        static unsigned long last_log = 0;
        if (millis() - last_log >= 100) {
            last_log = millis();
            float freq_step = vel_cmd_rad_s * (PASSOS_POR_VOLTA / (2.0f * PI_F));
            Serial.printf("ctrl=%s | pulsos=%6ld | ang=%6.2f° | tau=%+6.4f N.m | step=%+7.1f Hz\n",
                          controle_ativo ? "ON " : "OFF",
                          encoder_count, ang, tau, freq_step);
        }
    }
}
