/*
 * ============================================================================
 *  PÊNDULO INVERTIDO ROTACIONAL (FURUTA) — ESP32 + HW-134A (A4988) + ENCODER
 * ============================================================================
 *  Hardware:
 *    - ESP32 DevKit (30 pinos) na placa de bornes MRD068A
 *    - Driver HW-134A (módulo A4988) + motor Minebea 17PM-K044UG01LN
 *        1,8°/passo, 1,2 A/fase, 2,3 Ω, 2,2 mH, 200 mN·m, 37 g·cm² (unipolar, 6 fios)
 *    - Encoder E38S6G5-600B-G24: 600 PPR, 5–24 V, saída NPN coletor aberto
 *        vermelho +V | preto GND | verde A | branco B | malha → GND
 *    - Haste (antena telescópica): m = 8,5078 g, L = 29 cm
 *
 *  Especificações (degrau na referência do braço):
 *    ultrapassagem Mp = 15 %   |   tempo de acomodação ts = 3 s (2 %)
 *
 *  Modelo (entrada u = aceleração angular do braço, imposta pelo motor de passo):
 *        θ̈ = a·θ − b·u        α̈ = u
 *        a = (2π/T)²   (T = período da haste PENDURADA; ou 3g/(2L) se uniforme)
 *        b = a·r/g     (r = eixo do motor → plano da haste)
 *    Haste uniforme: a ≈ 50,7 s⁻² → polos de malha aberta em ±7,12 rad/s.
 *
 *  Controle em duas malhas, projetado pelo LUGAR DAS RAÍZES:
 *    malha interna (haste): PD   u1 = Kp_h·θ + Kd_h·θ̇
 *      zero em −7 e polo rápido em −60 (condição de módulo) → polos −60 e −7,08
 *    malha externa (braço): PID  u2 = Kp_b·(α − β·α_ref) + Ki_b·∫(α − α_ref)dt + Kd_b·α̇
 *      o par dominante (ζ = 0,54, ωn = 2,64 rad/s) e o polo do integrador (−3)
 *      são impostos como pontos do lugar das raízes (condições de ângulo e módulo)
 *    Polos de malha fechada: −13,0 ± j3,5; −1,43 ± j2,22; −3
 *      → Mp = 15,0 % e ts = 2,94 s no degrau do braço (modelo linear)
 *    O integrador mantém o braço centrado com erro de zero do encoder.
 *
 *  Requisitos: Arduino IDE + placa "esp32 by Espressif" versão 3.x
 *
 *  Comandos (Monitor Serial, 115200 baud, terminador "Nova linha"):
 *    m      painel legível: encoder e motor (padrão; m5 = 5 atualizações/s)
 *    p      saída para o Plotter Serial      s      silencia a saída periódica
 *    h      ajuda
 *    g      arma o equilíbrio           x      PARADA (desabilita o motor)
 *    e      reabilita o motor           z      zero do braço na posição atual
 *    c      zero do encoder (haste pendurada e parada)
 *    t      teste do motor (+90° e volta)
 *    j<n>   gira o braço n graus (ex.: j30, j-30)
 *    n      teste de micropasso: 100 passos devagar e tabela de quanto deveria girar
 *    v      CALIBRA sentidos (encoder × motor) e o raio do braço — rode antes de equilibrar
 *    w      mede o período da haste pendurada → identifica 'a', recalcula K
 *           e acha o ZERO verdadeiro (centro da oscilação)
 *    r<n>   referência do braço em graus (ex.: r45, r0)
 *    q<n>   ENSAIO DE DEGRAU de n graus: grava 8 s, imprime Mp, ts e CSV
 *    q<n>,<k> k degraus alternados (ida e volta) com resposta MEDIA (ex.: q60,6)
 *    k      mostra modelo, polos e ganhos
 * ============================================================================
 */

#include <Arduino.h>
#include "esp_arduino_version.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include <complex>

#if ESP_ARDUINO_VERSION_MAJOR < 3
#error "Este codigo requer o core ESP32 do Arduino versao 3.x (Ferramentas > Placa > Gerenciador de Placas)."
#endif

// ============================================================================
//  1) PINOS  —  ajuste conforme a sua ligação nos bornes
// ============================================================================
#define PIN_ENC_A     32   // fio VERDE do encoder  (pull-up p/ 3V3!)
#define PIN_ENC_B     33   // fio BRANCO do encoder (pull-up p/ 3V3!)
#define PIN_STEP      25   // driver STEP   (use GPIO < 32)
#define PIN_DIR       26   // driver DIR    (use GPIO < 32)
#define PIN_EN        27   // driver ENABLE (ativo em nível baixo). -1 se EN estiver fixo no GND

// ============================================================================
//  2) PARÂMETROS FÍSICOS
// ============================================================================
const float G_ACC         = 9.81f;
const float ROD_MASS      = 8.5078e-3f;   // kg (não entra em a e b; só documentação)
const float ROD_LEN       = 0.29f;        // m
const float ARM_RADIUS    = 0.0361f;      // m — eixo do motor → plano da haste (confirmado na bancada)
const float PEND_PERIOD_S = 0.9398f;      // s — medido com o comando w na bancada (06/11, amplitude 16°)

// ============================================================================
//  3) MOTOR / DRIVER
// ============================================================================
const int   MOTOR_STEPS_REV = 200;     // 17PM-K044: 1,8° por passo
const int   MICROSTEPS      = 2;      // A4988: MS1=MS2=MS3=VDD → 16 ; todos soltos → 1
const bool  DIR_INVERT      = true;
const float STEPS_PER_RAD   = MOTOR_STEPS_REV * MICROSTEPS / (2.0f * PI);

// ============================================================================
//  4) ENCODER
// ============================================================================
const int   ENC_PPR     = 600;
const int   ENC_CPR     = ENC_PPR * 4;          // 2400 contagens/volta (0,15°)
const int   ENC_LIMIT   = ENC_CPR * 10;         // múltiplo de CPR → o reset do PCNT preserva o ângulo
const bool  ENC_INVERT  = true;           // indicado pelo comando v na bancada (06/11)
const float RAD_PER_CNT = 2.0f * PI / ENC_CPR;

// ============================================================================
//  5) CONTROLE
// ============================================================================
const float CTRL_HZ   = 1000.0f;
const float DT        = 1.0f / CTRL_HZ;

// Controle em DUAS MALHAS, projetado pelo LUGAR DAS RAÍZES (veja computeGains):
//  (1) HASTE, malha interna — PD:  u1 = Kp_h·θ + Kd_h·θ̇
//      Zero em −PD_ZERO (logo à esquerda do polo estável −√a, sem cancelar) e ganho
//      alto, pela condição de módulo no polo real rápido INNER_FAST_POLE.
//  (2) BRAÇO, malha externa — PID: u2 = Kp_b·(α − β·α_ref) + Ki_b·∫(α − α_ref) + Kd_b·α̇
//      O par dominante (DOM_ZETA, DOM_WN) e o polo do integrador (POLE_INT) são
//      impostos como pontos do lugar das raízes. Como a planta do braço tem um zero
//      no semiplano direito, a realimentação do braço é POSITIVA (lugar de 0°).
//      β (peso da referência no termo proporcional) põe o zero da referência sobre
//      o polo do integrador, para ele não aparecer na resposta ao degrau.
// Os polos dominantes saem de Mp = 15 % e ts = 3 s (ζ = 0,517; ωn = 2,58) e foram
// ajustados para ζ = 0,54 e ωn = 2,64 por causa do zero de fase não mínima.
const float PD_ZERO         = 7.0f;    // zero do PD da haste (rad/s)
const float INNER_FAST_POLE = -60.0f;  // polo real rápido da malha interna
const float DOM_ZETA        = 0.54f;   // par dominante do braço
const float DOM_WN          = 2.64f;   // rad/s
const float POLE_INT        = -3.0f;   // polo do integrador
const float ZINT_MAX   = 1.5f;       // rad·s — limite do estado integral (anti-windup)

// Observador de estados da haste (θ̂, θ̂̇): usa o modelo e a aceleração do braço
// (conhecida) e o encoder só corrige, interpolando entre as contagens de 0,15°.
// Simulado: em 1/16 de passo ele reduz o "ciclo-limite" do braço de ~32° para ~2,4°
// pico a pico. Em meio passo (1/2) cada passo de 0,9° já sacode a haste e funciona
// como dither, e a derivada filtrada simples é melhor (ciclo ~6°, aguenta empurrões
// maiores). Por isso o observador liga automaticamente só com micropasso fino.
const bool  USE_OBSERVER = (MICROSTEPS >= 8);
const float OBS_P1 = -20.0f, OBS_P2 = -25.0f;   // polos do observador (rad/s)
const float VEL_FILTER_HZ = 40.0f;             // 15–25 Hz reduz a oscilação do braço (teste no Wokwi)
const float U_MAX         = 150.0f;             // rad/s² (o degrau de 90° usa < 30; acima disso o motor perde passo)
const float V_MAX         = 12.0f;              // rad/s
const float REF_MAX       = 90.0f * DEG_TO_RAD; // limite da referência do braço

// Auto-trim do zero do pêndulo. Funciona como uma ação integral lenta, então
// fica CONGELADO por TRIM_FREEZE_MS após cada mudança de referência para não
// alterar Mp/ts do ensaio de degrau.
const float    TRIM_GAIN      = 0.0f;     // DESLIGADO: o integrador do controle faz esse papel (0.008 = versão antiga)
const float    TRIM_MAX       = 10.0f * DEG_TO_RAD;  // inclui o zero achado pelo comando w
const uint32_t TRIM_FREEZE_MS = 10000;

// Segurança / máquina de estados
// Região de captura: com o braço limitado a ±120°, só é possível recuperar a haste
// se o modo instável x = θ + θ̇/ω (ω = √a) for menor que ~7° (simulado). Engatamos
// com margem, em 4°. Ex.: θ = 3° com θ̇ = 0,9 rad/s dá x = 10,7° → sem recuperação.
const float ENGAGE_ANGLE  = 6.0f   * DEG_TO_RAD;
const float ENGAGE_XU     = 4.0f   * DEG_TO_RAD;
const float ENGAGE_RATE   = 1.0f;
const uint32_t ENGAGE_MS  = 30;
const float FALL_ANGLE    = 30.0f  * DEG_TO_RAD;
const float ARM_LIMIT     = 360.0f * DEG_TO_RAD; // cabo do encoder!
const float MOVE_VMAX     = 3.0f;
const float MOVE_AMAX     = 30.0f;

// Gerador de passos
const uint32_t STEP_ISR_HZ = 40000;
const float    MAX_SPS     = STEP_ISR_HZ * 0.45f;

// Ensaio de degrau
const int      LOG_HZ      = 200;
const int      LOG_SECONDS = 8;
const int      LOG_N       = LOG_HZ * LOG_SECONDS;

// ============================================================================
//  ESTADO GLOBAL
// ============================================================================
static pcnt_unit_handle_t s_pcnt = nullptr;
static hw_timer_t*        s_timer = nullptr;

volatile int32_t g_rateHalf = 0;
volatile int32_t g_stepPos  = 0;

static uint32_t s_phase     = 0;
static bool     s_pulseHigh = false;
static bool     s_pending   = false;
static int8_t   s_dirCur    = 1;

enum State { ST_STOP, ST_IDLE, ST_BALANCE, ST_RETURN, ST_TEST, ST_CAL };
static State state = ST_IDLE;
static const char* stateName[] = {"STOP", "IDLE", "BALANCE", "RETURN", "TEST", "CAL"};

static float K1, K2, K3, K4, K5;
static float zInt = 0;               // ∫(α − α_r) dt
static float thHat = 0, thdHat = 0;  // observador da haste
static float armAcc = 0;             // aceleração do braço aplicada no último passo
static float betaRef = 1.0f;         // peso da referência no termo proporcional do braço
static float innerP1 = 0, innerP2 = 0;   // polos da malha interna (para o relatório)
static float clFastRe = 0, clFastIm = 0; // par rápido de malha fechada (idem)
static float modelA, modelB;
static float  armRadius = ARM_RADIUS;   // pode ser recalibrado com o comando 'v'
static int8_t encSign   = 1;            // idem (inverte o sentido do encoder em software)

// Calibração automática de sentido e raio (comando 'v')
const float CAL_V   = 2.0f;             // rad/s  — velocidade do "tranco" no braço
const float CAL_ACC = 100.0f;           // rad/s² — aceleração até CAL_V (20 ms; 200 fazia o motor perder passo)
const float CAL_WIN = 0.25f;            // s      — janela para o pico da haste (> 1/4 de período)
static uint8_t  calPhase = 0;
static uint32_t calT0    = 0;
static int32_t  calPhi0  = 0, calPeak = 0;

static int32_t  stepZero    = 0;
static float    vCmd = 0, uCmd = 0;
static float    theta = 0, thetaDot = 0, alpha = 0, thetaFromDown = 0;
static float    alphaRef    = 0;
static float    thetaOffset = 0;
static uint32_t trimFreezeUntil = 0;
static int32_t  prevCount   = 0;
static bool     filterInit  = false;
static uint32_t engageStart = 0;
static uint8_t  testPhase   = 0;
static uint32_t testTimer   = 0;
static float    jogTarget   = 0;
static int32_t  testStart   = 0;
static bool     motorEnabled = false;

// Saída serial
enum OutMode { OUT_MONITOR, OUT_PLOTTER, OUT_SILENT };
static OutMode  outMode     = OUT_MONITOR;    // padrão: painel legível no Monitor Serial
static uint32_t monPeriodMs = 500;            // painel a 2 Hz (m1..m10 muda)

// Diagnóstico do encoder (amostrado no laço de 1 kHz)
static uint8_t  encLevelA = 0, encLevelB = 0;
static uint32_t encTogA = 0, encTogB = 0;     // transições de cada canal na janela atual
static bool     encSawA = false, encSawB = false;  // já viu os dois canais mudarem desde o boot
static bool     encFault = false;

// Ensaio de degrau
static float*   logAcc     = nullptr;  // soma das respostas normalizadas
static int      logIdx     = -1;       // -1 = sem ensaio
static float    logStepDeg = 0, logStartDeg = 0;   // 1º degrau (para o relatório)
static float    segFrom = 0, segTo = 0;            // degrau em andamento
static int      logDiv     = 0;
static int      logRuns = 0, logRunsTotal = 1;

// Medição do período
static bool     perActive  = false;
static bool     perArmed   = false;
static int      perCount   = 0;
static uint32_t perT0 = 0, perTLast = 0, perStart = 0;
static float    perAmp     = 0;
static int32_t  perMax = 0, perMin = 0;      // extremos do ciclo atual (para achar o zero)
static float    perPrevMin = 0;
static bool     perHavePrevMin = false;
static float    perCenterSum = 0;
static int      perCenterN = 0;
static uint32_t balanceStartMs = 0;
static float    engTh = 0, engThd = 0;      // estado da haste no engate

// ----------------------------------------------------------------------------
//  "Caixa-preta" do engate: estima, nos primeiros 0,6 s de equilíbrio, quanto o
//  movimento do braço realmente age na haste. Modelo: θ̈ = a·sen θ − β·cos θ·α̈.
//  Integrando duas vezes: θ(t) − θ0 − a·∬sen θ = θ̇0·t − β·∬cos θ·α̈  → mínimos
//  quadrados em (θ̇0, β), acumulado online (sem guardar vetores).
//  β/b ≈ 1: acoplamento OK | ≈ 0: o braço não move a haste | ≈ −1: sentidos invertidos
// ----------------------------------------------------------------------------
const float  EST_WINDOW = 0.6f;     // s
static bool  estOn = false, estHave0 = false;
static int   estN = 0;
static float estT = 0, estTh0 = 0;
static double estS1s, estS2s, estS1c, estS2c, estStt, estStH, estSHH, estSty, estSHy;
const  int      PER_CYCLES = 6;

// ============================================================================
//  GERADOR DE PASSOS (ISR a 40 kHz, acumulador de fase)
// ============================================================================
#define STEP_MASK (1UL << PIN_STEP)
#define DIR_MASK  (1UL << PIN_DIR)

void ARDUINO_ISR_ATTR onStepTimer() {
  bool justCleared = false;
  if (s_pulseHigh) {                         // pulso de 25 µs
    REG_WRITE(GPIO_OUT_W1TC_REG, STEP_MASK);
    s_pulseHigh = false;
    justCleared = true;
  }

  int32_t r    = g_rateHalf;
  int8_t  dir  = (r >= 0) ? 1 : -1;
  uint32_t inc = (uint32_t)(r >= 0 ? r : -r) << 1;

  if (dir != s_dirCur) {                     // troca de sentido: 1 tick de setup
    bool level = (dir > 0) ^ DIR_INVERT;
    REG_WRITE(level ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, DIR_MASK);
    s_dirCur  = dir;
    s_pending = false;
    return;
  }

  uint32_t old = s_phase;
  s_phase += inc;
  if (s_phase < old) s_pending = true;       // estouro = 1 passo

  if (s_pending && !justCleared) {
    REG_WRITE(GPIO_OUT_W1TS_REG, STEP_MASK);
    s_pulseHigh = true;
    s_pending   = false;
    g_stepPos  += s_dirCur;
  }
}

static void setArmVelocity(float v) {
  float sps = constrain(v * STEPS_PER_RAD, -MAX_SPS, MAX_SPS);
  g_rateHalf = (int32_t)(sps * (2147483648.0f / (float)STEP_ISR_HZ));
}

static void motorEnable(bool on) {
  motorEnabled = on;
  if (PIN_EN >= 0) digitalWrite(PIN_EN, on ? LOW : HIGH);
}

static float armAngle() {
  return (float)(g_stepPos - stepZero) / STEPS_PER_RAD;
}

// ============================================================================
//  ENCODER (PCNT)
// ============================================================================
static bool encoderBegin() {
  pcnt_unit_config_t ucfg = {};
  ucfg.low_limit  = -ENC_LIMIT;
  ucfg.high_limit =  ENC_LIMIT;
  if (pcnt_new_unit(&ucfg, &s_pcnt) != ESP_OK) return false;

  pcnt_glitch_filter_config_t fcfg = {};
  fcfg.max_glitch_ns = 1000;
  pcnt_unit_set_glitch_filter(s_pcnt, &fcfg);

  pcnt_chan_config_t ca = {};
  ca.edge_gpio_num  = ENC_INVERT ? PIN_ENC_B : PIN_ENC_A;
  ca.level_gpio_num = ENC_INVERT ? PIN_ENC_A : PIN_ENC_B;
  pcnt_chan_config_t cb = {};
  cb.edge_gpio_num  = ca.level_gpio_num;
  cb.level_gpio_num = ca.edge_gpio_num;

  pcnt_channel_handle_t chA = nullptr, chB = nullptr;
  if (pcnt_new_channel(s_pcnt, &ca, &chA) != ESP_OK) return false;
  if (pcnt_new_channel(s_pcnt, &cb, &chB) != ESP_OK) return false;

  pcnt_channel_set_edge_action (chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP,    PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action (chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP,    PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_unit_add_watch_point(s_pcnt,  ENC_LIMIT);
  pcnt_unit_add_watch_point(s_pcnt, -ENC_LIMIT);

  // Saída NPN coletor aberto: o pull-up define o nível alto. Os internos (~45 kΩ)
  // funcionam, mas use 2,2–4,7 kΩ externos para 3V3 (NUNCA para 5 V).
  // GPIO 34–39 não têm pull-up interno.
  gpio_pullup_en((gpio_num_t)PIN_ENC_A);
  gpio_pullup_en((gpio_num_t)PIN_ENC_B);

  pcnt_unit_enable(s_pcnt);
  pcnt_unit_clear_count(s_pcnt);
  pcnt_unit_start(s_pcnt);
  return true;
}

static int32_t encoderRaw() {
  int c = 0;
  pcnt_unit_get_count(s_pcnt, &c);
  return encSign * c;
}

// Contagens relativas à posição PENDURADA, em [-1200, 1199]
static int32_t countsFromDown(int32_t count) {
  int32_t m = count % ENC_CPR;
  if (m < 0) m += ENC_CPR;
  return (m >= ENC_CPR / 2) ? m - ENC_CPR : m;
}

// Ângulo a partir da VERTICAL PARA CIMA, em [-π, π)
static float pendulumAngle(int32_t count) {
  int32_t m = count % ENC_CPR;
  if (m < 0) m += ENC_CPR;
  return (float)(m - ENC_CPR / 2) * RAD_PER_CNT;
}

// ============================================================================
//  MODELO E GANHOS — projeto pelo lugar das raízes, em duas malhas
//  Planta:  Θ/U = −b/(s² − a)   e   A/U = 1/s²   (u = aceleração do braço)
//
//  Malha interna (haste), PD com zero em −z1:
//     equação característica  s² − a + b·Kd_h·(s + z1) = 0
//     condição de módulo no polo real pf:  b·Kd_h = |pf − √a|·|pf + √a| / |pf + z1|
//
//  Malha externa (braço), PID  C(s) = Kd_b·s + Kp_b + Ki_b/s  (realimentação positiva):
//     planta vista pelo PID:  P2(s) = (s² − a) / (s²·(s² + i1·s + i0))
//     equação característica  1 − C(s)·P2(s) = 0   →   C(s) = 1/P2(s)
//     impondo essa igualdade em s0 (complexo: ângulo e módulo) e em p3 (real),
//     obtêm-se três equações lineares em Kd_b, Kp_b e Ki_b.
//
//  O firmware guarda K = [K1..K5] com u = −(K1θ + K2θ̇ + K3·eα + K4α̇ + K5∫), por isso
//  K1 = −Kp_h, K2 = −Kd_h, K3 = −Kp_b, K4 = −Kd_b, K5 = −Ki_b.
// ============================================================================
static void computeGains(float a) {
  modelA = a;
  modelB = a * armRadius / G_ACC;
  const double A = a, B = modelB, ra = sqrt(A);

  // --- malha interna: PD da haste ---
  const double pf = INNER_FAST_POLE, z1 = PD_ZERO;
  double bKd = fabs(pf - ra) * fabs(pf + ra) / fabs(pf + z1);   // condição de módulo
  double KdH = bKd / B, KpH = KdH * z1;
  double i1 = bKd, i0 = bKd * z1 - A;                           // s² + i1·s + i0
  double disc = i1 * i1 - 4.0 * i0;
  innerP1 = (float)((-i1 - sqrt(fabs(disc))) / 2.0);
  innerP2 = (float)((-i1 + sqrt(fabs(disc))) / 2.0);

  // --- malha externa: PID do braço ---
  const double sg = -DOM_ZETA * DOM_WN, wd = DOM_WN * sqrt(1.0 - DOM_ZETA * DOM_ZETA);
  const std::complex<double> s0(sg, wd);
  const double p3 = POLE_INT;
  std::complex<double> R0 = s0 * s0 * s0 * (s0 * s0 + i1 * s0 + i0) / (s0 * s0 - A);  // s·C(s0)
  double R3 = p3 * p3 * p3 * (p3 * p3 + i1 * p3 + i0) / (p3 * p3 - A);                 // s·C(p3)
  // Kd·s² + Kp·s + Ki = R(s) em s0 (parte real e imaginária) e em p3
  std::complex<double> s02 = s0 * s0;
  double a11 = s02.real() - p3 * p3, a12 = sg - p3, b1 = R0.real() - R3;
  double a21 = s02.imag(),           a22 = wd,      b2 = R0.imag();
  double det = a11 * a22 - a12 * a21;
  double KdB = (b1 * a22 - a12 * b2) / det;
  double KpB = (a11 * b2 - a21 * b1) / det;
  double KiB = R3 - KdB * p3 * p3 - KpB * p3;
  betaRef = (float)(KiB / (KpB * fabs(p3)));

  // par rápido de malha fechada: sai da soma e do produto das raízes do polinômio de 5º grau
  double sumFast  = -(i1 - KdB) - 2.0 * sg - p3;                // soma das 5 raízes = −(i1 − Kd_b)
  double prodFast = (A * KiB) / ((sg * sg + wd * wd) * (-p3));  // produto das 5 raízes = −a·Ki_b
  double dF = sumFast * sumFast / 4.0 - prodFast;
  clFastRe = (float)(sumFast / 2.0);
  clFastIm = (dF < 0) ? (float)sqrt(-dF) : 0.0f;

  K1 = (float)(-KpH);  K2 = (float)(-KdH);
  K3 = (float)(-KpB);  K4 = (float)(-KdB);  K5 = (float)(-KiB);
}

static float unstableMode() {          // x = θ + θ̇/ω  (rad)
  return theta + thetaDot / sqrtf(modelA);
}

static float defaultA() {
  if (PEND_PERIOD_S > 0) {
    float w = 2.0f * PI / PEND_PERIOD_S;
    return w * w;
  }
  return 3.0f * G_ACC / (2.0f * ROD_LEN);
}

static void printModel() {
  const float sg = -DOM_ZETA * DOM_WN, wd = DOM_WN * sqrtf(1.0f - DOM_ZETA * DOM_ZETA);
  Serial.println(F("\n--- Modelo e ganhos (lugar das raizes) ---"));
  Serial.printf("a = %.3f 1/s^2  (polos MA do pendulo: +/-%.3f rad/s)\n", modelA, sqrtf(modelA));
  Serial.printf("b = %.4f        (r = %.1f mm)\n", modelB, armRadius * 1000);
  Serial.printf("Malha interna (haste, PD): zero em -%.2f | polos %.2f e %.2f\n", PD_ZERO, innerP1, innerP2);
  Serial.printf("   Kp = %.1f | Kd = %.1f\n", -K1, -K2);
  Serial.printf("Malha externa (braco, PID): polos dominantes %.3f +/- j%.3f (zeta = %.2f) | integrador %.2f\n",
                sg, wd, DOM_ZETA, POLE_INT);
  Serial.printf("   Kp = %.2f | Ki = %.2f | Kd = %.2f | beta = %.3f\n", -K3, -K5, -K4, betaRef);
  Serial.printf("Polos rapidos de malha fechada: %.2f +/- j%.2f\n", clFastRe, clFastIm);
  if (USE_OBSERVER) Serial.printf("Velocidade da haste: observador de estados (polos %.0f, %.0f)\n", OBS_P1, OBS_P2);
  else              Serial.printf("Velocidade da haste: derivada filtrada a %.0f Hz (meio passo: observador desligado)\n", VEL_FILTER_HZ);
  Serial.printf("K = [%.2f  %.2f  %.3f  %.3f  %.3f]  (theta, thetaDot, alfa, alfaDot, integral)\n",
                K1, K2, K3, K4, K5);
  Serial.printf("Micropasso 1/%d -> %.1f passos/rad\n", MICROSTEPS, STEPS_PER_RAD);
  Serial.printf("Auto-trim: %.3f graus | ref. braco: %.1f graus\n\n",
                thetaOffset * RAD_TO_DEG, alphaRef * RAD_TO_DEG);
}

static void printHelp() {
  Serial.println(F("\n================================ COMANDOS ================================"));
  Serial.println(F(" Saida   : m  painel legivel (m5 = 5 vezes/s) | p  Plotter Serial | s  silencio"));
  Serial.println(F(" Motor   : t  teste +90 graus e volta | j<graus> gira (ex.: j30, j-30) | n  teste de micropasso"));
  Serial.println(F("           x  desliga o motor | e  liga o motor | z  zero do braco aqui"));
  Serial.println(F(" Encoder : c  zero (haste pendurada e parada) | w  periodo + zero verdadeiro (empurre a haste)"));
  Serial.println(F(" Calibrar: v  sentidos + raio do braco (haste pendurada e parada)"));
  Serial.println(F(" Controle: g  armar equilibrio | r<graus> referencia | q<graus>[,n] ensaio"));
  Serial.println(F("           k  modelo e ganhos | h  esta ajuda"));
  Serial.println(F(" (Monitor Serial: 115200 baud, terminador \"Nova linha\")"));
  Serial.println(F("==========================================================================\n"));
}

// ============================================================================
//  MOVIMENTO SIMPLES DO BRAÇO
// ============================================================================
static bool moveToward(float target) {
  float err  = target - alpha;
  float vDes = constrain(4.0f * err, -MOVE_VMAX, MOVE_VMAX);
  vCmd += constrain(vDes - vCmd, -MOVE_AMAX * DT, MOVE_AMAX * DT);
  setArmVelocity(vCmd);
  return fabsf(err) < 0.3f * DEG_TO_RAD && fabsf(vCmd) < 0.05f;
}

static void gotoState(State s) {
  if (s == ST_BALANCE) {
    vCmd = 0; filterInit = false; alphaRef = 0; balanceStartMs = millis(); zInt = 0;
    thHat = theta; thdHat = thetaDot; armAcc = 0;
    estOn = true; estHave0 = false; estN = 0; estT = 0;
    estS1s = estS2s = estS1c = estS2c = estStt = estStH = estSHH = estSty = estSHy = 0;
  }
  if (s == ST_STOP)    { vCmd = 0; setArmVelocity(0); motorEnable(false); }
  if (s == ST_IDLE)    { vCmd = 0; setArmVelocity(0); engageStart = 0; }
  if (s != ST_BALANCE) { logIdx = -1; alphaRef = 0; }
  state = s;
  Serial.printf("# estado -> %s\n", stateName[s]);
}

static void setReference(float deg) {
  deg = constrain(deg, -REF_MAX * RAD_TO_DEG, REF_MAX * RAD_TO_DEG);
  alphaRef = deg * DEG_TO_RAD;
  trimFreezeUntil = millis() + TRIM_FREEZE_MS;
}

// ============================================================================
//  ENSAIO DE DEGRAU: resultado
// ============================================================================
// O relatório NÃO pode bloquear o laço (o pêndulo continua equilibrando):
// o resumo é calculado de uma vez e o CSV sai algumas linhas por vez em loop().
static int reportIdx = -1;

// logAcc guarda a SOMA das respostas normalizadas y = (α − α_inicial)/(α_final − α_inicial)
// de todos os degraus do ensaio; a média reduz a oscilação aleatória causada pela
// resolução do encoder (0,15°), que no braço vira ±2–3°.
static float yAvg(int i) { return logAcc[i] / logRuns; }

static void reportStepTest() {
  float yMax = -1e9;
  for (int i = 0; i < LOG_N; i++) if (yAvg(i) > yMax) yMax = yAvg(i);
  // ts: último instante fora da faixa (2 % e 5 %), com média móvel de 0,1 s
  const int W = LOG_HZ / 10;
  float ts2 = 0, ts5 = 0;
  for (int i = W; i < LOG_N; i++) {
    float s = 0;
    for (int j = i - W; j < i; j++) s += yAvg(j);
    float y = s / W;
    float t = (float)(i - W / 2) / LOG_HZ;
    if (fabsf(y - 1.0f) > 0.02f) ts2 = t;
    if (fabsf(y - 1.0f) > 0.05f) ts5 = t;
  }
  Serial.println(F("# ===== RESULTADO ====="));
  Serial.printf("# Degrau: %.1f <-> %.1f graus, media de %d degrau(s)\n",
                logStartDeg, logStartDeg + logStepDeg, logRuns);
  Serial.printf("# Ultrapassagem Mp = %.1f %%   (especificado: 15 %%)\n", (yMax - 1.0f) * 100.0f);
  Serial.printf("# Acomodacao ts(2%%) = %.2f s | ts(5%%) = %.2f s   (especificado: 3 s)\n", ts2, ts5);
  if (ts2 >= LOG_SECONDS - 0.2f)
    Serial.println(F("# Obs.: nao entrou na faixa de 2% (oscilacao de quantizacao). Use mais repeticoes: q60,6"));
  Serial.println(F("# ===== CSV (copie a partir da proxima linha) ====="));
  Serial.println(F("t_s,y_normalizado,alpha_equivalente_deg"));
  reportIdx = 0;
}

static void reportStream() {                    // chamado a cada volta do loop()
  if (reportIdx < 0) return;
  while (reportIdx < LOG_N && Serial.availableForWrite() > 48) {
    float y = yAvg(reportIdx);
    Serial.printf("%.3f,%.4f,%.3f\n", (float)reportIdx / LOG_HZ, y, logStartDeg + y * logStepDeg);
    reportIdx++;
  }
  if (reportIdx >= LOG_N) {
    Serial.println(F("# ===== fim do CSV =====\n"));
    reportIdx = -1;
  }
}

// ============================================================================
//  MEDIÇÃO DO PERÍODO (haste pendurada, motor parado)
// ============================================================================
static void periodUpdate(int32_t cnt) {
  if (!perActive) return;
  int32_t c = countsFromDown(cnt);
  float phiDeg = c * RAD_PER_CNT * RAD_TO_DEG;
  if (fabsf(phiDeg) > perAmp) perAmp = fabsf(phiDeg);
  uint32_t now = micros();

  if (millis() - perStart > 30000) {
    perActive = false;
    Serial.println(F("# periodo: tempo esgotado (de um empurraozinho na haste pendurada)"));
    return;
  }
  if (perCount >= 1) {                          // extremos dentro do ciclo
    if (c > perMax) perMax = c;
    if (c < perMin) perMin = c;
  }
  if (c < -3) perArmed = true;                  // histerese de 3 contagens
  if (perArmed && c >= 0) {                     // cruzamento ascendente pelo zero
    perArmed = false;
    // Centro da oscilação = repouso VERDADEIRO. Usa max_k e a média de min_(k-1)
    // e min_k, o que cancela o decaimento da amplitude causado pelo atrito.
    if (perCount >= 1) {
      if (perHavePrevMin) {
        perCenterSum += 0.5f * (perMax + 0.5f * (perPrevMin + perMin));
        perCenterN++;
      }
      perPrevMin = perMin; perHavePrevMin = true;
    }
    perMax = perMin = c;
    if (perCount == 0) perT0 = now;
    perTLast = now;
    perCount++;
    if (perCount > PER_CYCLES) {
      perActive = false;
      float T  = (perTLast - perT0) * 1e-6f / PER_CYCLES;
      float w  = 2.0f * PI / T;
      float aMeas = w * w;
      float aUni  = 3.0f * G_ACC / (2.0f * ROD_LEN);
      Serial.printf("\n# Periodo medido T = %.4f s (amplitude max %.1f graus)\n", T, perAmp);
      Serial.printf("# a medido = %.3f 1/s^2  (haste uniforme: %.3f)\n", aMeas, aUni);
      if (perAmp > 20.0f) Serial.println(F("# Atencao: amplitude grande; repita com oscilacao menor (< 15 graus)"));
      computeGains(aMeas);
      Serial.printf("# Para fixar, coloque no codigo: const float PEND_PERIOD_S = %.4ff;\n", T);
      if (perCenterN > 0) {
        float cen = perCenterSum / perCenterN;                 // em contagens
        thetaOffset = constrain(cen * RAD_PER_CNT, -TRIM_MAX, TRIM_MAX);
        Serial.printf("# ZERO: o centro da oscilacao esta a %+.2f graus da referencia do boot -> corrigido.\n",
                      cen * RAD_PER_CNT * RAD_TO_DEG);
        if (fabsf(cen * RAD_PER_CNT * RAD_TO_DEG) > 1.0f)
          Serial.println(F("#       (a haste leve para fora do centro por atrito do encoder; sem isso o braco deriva ate o limite)"));
      }
      printModel();
    }
  }
}

// ============================================================================
//  CALIBRAÇÃO DE SENTIDO E RAIO (comando 'v')
//  Com a haste PENDURADA, o braço leva um "tranco" de 0 a CAL_V em 10 ms.
//  Pendurada (cos θ = −1), a dinâmica vira φ̈ = −a·φ + b·α̈ (φ = desvio do repouso):
//    - o PRIMEIRO movimento da haste tem o sinal de b → confere os sentidos;
//    - o pico vale φmax = b·CAL_V/√a → estima b e, com b = a·r/g, o raio r.
//  Se os sentidos estiverem trocados, o controle empurra a haste para o lado
//  errado e ela nunca fica em pé — é a causa mais comum de "não equilibra".
// ============================================================================
static void calResult() {
  float phiPk = calPeak * RAD_PER_CNT;
  Serial.println(F("\n=============== CALIBRACAO (v) ==============="));
  Serial.printf("Pico da haste: %+ld contagens (%+.2f graus) com o braco a %+.1f rad/s\n",
                (long)calPeak, phiPk * RAD_TO_DEG, CAL_V);
  if (abs(calPeak) < 4) {
    Serial.println(F("FALHA: a haste quase nao se mexeu. O motor girou mesmo?"));
    Serial.println(F("       Confira VMOT, Vref, bobinas e se o braco nao esta travado."));
    Serial.println(F("==============================================\n"));
    return;
  }
  if (calPeak > 0) {
    Serial.println(F("Sentidos: OK (encoder e motor concordam)"));
  } else {
    encSign = -encSign;
    prevCount = encoderRaw();
    Serial.println(F("Sentidos: INVERTIDOS -> corrigido agora (encoder invertido em software)."));
    Serial.printf("          Para fixar, no codigo: const bool ENC_INVERT = %s;\n",
                  (ENC_INVERT ^ (encSign < 0)) ? "true" : "false");
  }
  float bEst = fabsf(phiPk) * sqrtf(modelA) / CAL_V;
  float rEst = bEst * G_ACC / modelA;
  Serial.printf("Raio do braco estimado: r = %.1f mm (no codigo: %.1f mm)\n", rEst * 1000, ARM_RADIUS * 1000);
  float f = rEst / ARM_RADIUS;                  // quanto a haste reagiu além do previsto
  if (f > 1.7f || f < 0.6f) {
    // r não muda tanto assim: o mais provável é o braço girar MAIS (ou menos) que o
    // comandado, ou seja, o micropasso do driver diferente de MICROSTEPS.
    int realMs = (int)lroundf(MICROSTEPS / f);
    int pow2 = 1; while (pow2 * 2 <= realMs * 1.41f && pow2 < 16) pow2 *= 2;
    Serial.printf("ATENCAO: a haste reagiu %.1fx o previsto. O braco provavelmente gira %.0fx MAIS que o comandado:\n", f, f);
    Serial.printf("         o driver deve estar em 1/%d de passo, e nao em 1/%d como no codigo.\n", pow2, MICROSTEPS);
    Serial.println(F("         Confirme com o comando n (gira 100 passos devagar e diz quanto deveria girar)."));
    Serial.println(F("         Correcao recomendada: ligue MS1, MS2 e MS3 do driver ao VDD (1/16 de passo)."));
    Serial.println(F("==============================================\n"));
    return;
  }
  if (rEst > 0.010f && rEst < 0.200f) {
    armRadius = rEst;
    computeGains(modelA);
    Serial.printf("          Aplicado e ganhos recalculados. Para fixar: const float ARM_RADIUS = %.4ff;\n", rEst);
  } else {
    Serial.println(F("          Valor fora do esperado (10-200 mm): mantido o do codigo. Meca r com a regua."));
  }
  if (PEND_PERIOD_S <= 0)
    Serial.println(F("Dica: rode 'w' antes de 'v' para medir 'a' de verdade (melhora a estimativa de r)."));
  Serial.println(F("==============================================\n"));
}

// ============================================================================
//  DIAGNÓSTICO DA FALHA pelo acoplamento estimado (caixa-preta)
// ============================================================================
static void reportCoupling(float tEng) {
  if (estN < 80) {
    Serial.println(F("#   (falhou rapido demais para estimar o acoplamento: < 0,08 s de dados)"));
    return;
  }
  double det = estStt * estSHH - estStH * estStH;
  if (fabs(det) < 1e-30) { Serial.println(F("#   (dados insuficientes para estimar o acoplamento)")); return; }
  double thd0  = (estSty * estSHH - estStH * estSHy) / det;
  double beta  = -(estStt * estSHy - estStH * estSty) / det;
  float  ratio = (float)(beta / modelB);
  float  rEst  = ratio * armRadius;
  Serial.printf("#   Caixa-preta (%.2f s de dados): o braco age na haste com %.0f%% do previsto",
                fminf(tEng, EST_WINDOW), ratio * 100.0f);
  Serial.printf(" (r efetivo = %.0f mm) | velocidade da haste ao engatar = %+.2f rad/s\n",
                rEst * 1000.0f, (float)thd0);
  if (ratio < -0.3f) {
    Serial.println(F("#   Diagnostico: SENTIDOS INVERTIDOS (o braco empurra a haste para o lado errado)."));
    Serial.println(F("#   -> Com a haste pendurada e parada, rode 'v' (ele corrige)."));
  } else if (ratio < 0.15f) {
    Serial.println(F("#   Diagnostico: o movimento do braco NAO chegou a haste. Ela se moveu como se o braco estivesse parado."));
    Serial.println(F("#   Causas, da mais provavel: (1) o motor nao girou de verdade ou perdeu passos:"));
    Serial.println(F("#      VMOT ligado? SLEEP e RESET do driver ligados juntos? Vref ajustado? bobinas certas?"));
    Serial.println(F("#   (2) o acoplamento motor->suporte do encoder patinou;  (3) a mao ainda segurava a haste."));
    Serial.println(F("#   -> Teste: envie j30 e veja se o braco gira 30 graus de verdade; depois 'v'."));
  } else if (ratio < 0.6f) {
    Serial.println(F("#   Diagnostico: o braco age na haste bem MENOS que o previsto."));
    Serial.printf("#   -> O raio real pode ser menor (ARM_RADIUS = %.4ff), ou ha atrito alto no eixo do encoder.\n", rEst);
    Serial.println(F("#      Rode 'v' para medir o raio. Se 'v' der ~45 mm, o problema e atrito/folga no encoder."));
  } else if (ratio > 1.7f) {
    Serial.printf("#   Diagnostico: o braco age MAIS que o previsto. Raio maior? (ARM_RADIUS = %.4ff) ou micropasso diferente de 1/%d.\n",
                  rEst, MICROSTEPS);
  } else {
    Serial.println(F("#   Acoplamento OK (sentidos, raio e motor coerentes)."));
    Serial.println(F("#   -> Causa provavel: zero do encoder (rode 'w') ou haste solta com velocidade alta."));
  }
}

// ============================================================================
//  LAÇO DE CONTROLE (1 kHz)
// ============================================================================
static void controlStep() {
  int32_t cnt = encoderRaw();
  float thMeas = pendulumAngle(cnt);
  thetaFromDown = countsFromDown(cnt) * RAD_PER_CNT;

  int32_t dc = cnt - prevCount;                 // wrap do PCNT em ±ENC_LIMIT
  if (dc >  ENC_CPR / 2) dc -= ENC_CPR * ((dc + ENC_CPR / 2) / ENC_CPR);
  if (dc < -ENC_CPR / 2) dc += ENC_CPR * ((-dc + ENC_CPR / 2) / ENC_CPR);
  prevCount = cnt;
  static const float kf = 1.0f - expf(-2.0f * PI * VEL_FILTER_HZ * DT);
  float rawVel = dc * RAD_PER_CNT / DT;
  if (!filterInit) { thetaDot = 0; filterInit = true; }
  else thetaDot += kf * (rawVel - thetaDot);

  theta = thMeas - thetaOffset;
  alpha = armAngle();

  // Níveis dos canais A e B (para o diagnóstico do encoder)
  uint8_t la = gpio_get_level((gpio_num_t)PIN_ENC_A);
  uint8_t lb = gpio_get_level((gpio_num_t)PIN_ENC_B);
  if (la != encLevelA) { encTogA++; encSawA = true; }
  if (lb != encLevelB) { encTogB++; encSawB = true; }
  encLevelA = la; encLevelB = lb;

  periodUpdate(cnt);

  switch (state) {
    case ST_STOP:
      break;

    case ST_IDLE:
      if (!motorEnabled || perActive) break;
      if (fabsf(theta) < ENGAGE_ANGLE && fabsf(thetaDot) < ENGAGE_RATE &&
          fabsf(unstableMode()) < ENGAGE_XU) {
        if (engageStart == 0) engageStart = millis();
        if (millis() - engageStart > ENGAGE_MS) { engTh = theta; engThd = thetaDot; gotoState(ST_BALANCE); }
      } else {
        engageStart = 0;
      }
      break;

    case ST_BALANCE: {
      if (fabsf(theta) > FALL_ANGLE || fabsf(alpha) > ARM_LIMIT) {
        float tEng = (millis() - balanceStartMs) * 1e-3f;
        Serial.printf("# FALHA apos %.2f s de equilibrio: desvio=%.1f graus, braco=%.1f graus, trim=%.2f graus\n",
                      tEng, theta * RAD_TO_DEG, alpha * RAD_TO_DEG, thetaOffset * RAD_TO_DEG);
        Serial.printf("#   No engate: desvio=%+.1f graus, velocidade=%+.2f rad/s -> modo instavel x=%+.1f graus (captura < 7)\n",
                      engTh * RAD_TO_DEG, engThd, (engTh + engThd / sqrtf(modelA)) * RAD_TO_DEG);
        if (tEng < 1.0f) {
          reportCoupling(tEng);
        } else if (fabsf(theta) > FALL_ANGLE) {
          Serial.println(F("#   Diagnostico: a haste caiu depois de equilibrar (empurrao forte, raio r ou micropasso errados)."));
        } else {
          Serial.printf("#   Diagnostico: o braco chegou ao limite de %.0f graus (perturbacao grande demais para o curso do braco).\n",
                        ARM_LIMIT * RAD_TO_DEG);
        }
        gotoState(ST_RETURN);
        break;
      }
      // --- erro do braço: o integral usa (α − α_ref); o proporcional usa (α − β·α_ref) ---
      float eAlpha = alpha - alphaRef;
      float ePropA = alpha - betaRef * alphaRef;
      // --- observador: θ̂' = θ̂̇ + L1·e ; θ̂̇' = a·θ̂ − b·α̈ + L2·e ---
      {
        const float L1 = -(OBS_P1 + OBS_P2), L2 = OBS_P1 * OBS_P2 + modelA;
        float ey = theta - thHat;
        thHat  += DT * (thdHat + L1 * ey);
        thdHat += DT * (modelA * thHat - modelB * armAcc + L2 * ey);
      }
      float thUse  = USE_OBSERVER ? thHat  : theta;
      float thdUse = USE_OBSERVER ? thdHat : thetaDot;
      // PD da haste (K1, K2) + PID do braço (K3 proporcional, K4 derivativo, K5 integral)
      float uRaw = -(K1 * thUse + K2 * thdUse + K3 * ePropA + K4 * vCmd + K5 * zInt);
      float u = constrain(uRaw, -U_MAX, U_MAX);
      uCmd = u;
      // anti-windup: não integra quando saturado e a integração pioraria a saturação
      bool sat = fabsf(uRaw) > U_MAX;
      if (!(sat && ((-K5 * eAlpha > 0) == (uRaw > 0))))
        zInt = constrain(zInt + eAlpha * DT, -ZINT_MAX, ZINT_MAX);
      float vOld = vCmd;
      vCmd = constrain(vCmd + u * DT, -V_MAX, V_MAX);
      setArmVelocity(vCmd);

      armAcc = (vCmd - vOld) / DT;
      if (estOn) {                               // caixa-preta (veja reportCoupling)
        float accArm = armAcc;
        if (!estHave0) { estTh0 = theta; estHave0 = true; }
        else {
          estT += DT;
          double y = theta - estTh0 - modelA * estS2s;
          double H = estS2c;
          estStt += (double)estT * estT; estStH += estT * H; estSHH += H * H;
          estSty += estT * y;            estSHy += H * y;
          estN++;
        }
        estS1s += sinf(theta) * DT;            estS2s += estS1s * DT;
        estS1c += cosf(theta) * accArm * DT;   estS2c += estS1c * DT;
        if (estT >= EST_WINDOW) estOn = false;
      }

      if ((int32_t)(millis() - trimFreezeUntil) >= 0) {
        thetaOffset -= TRIM_GAIN * eAlpha * DT;
        thetaOffset = constrain(thetaOffset, -TRIM_MAX, TRIM_MAX);
      }

      if (logIdx >= 0 && logIdx < LOG_N) {      // gravação do ensaio
        if (++logDiv >= (int)(CTRL_HZ / LOG_HZ)) {
          logDiv = 0;
          logAcc[logIdx++] += (alpha * RAD_TO_DEG - segFrom) / (segTo - segFrom);
        }
      }
      break;
    }

    case ST_CAL: {
      float tc = (micros() - calT0) * 1e-6f;
      if (calPhase == 0) {                      // tranco + observação do pico da haste
        vCmd = fminf(vCmd + CAL_ACC * DT, CAL_V);
        setArmVelocity(vCmd);
        int32_t dphi = countsFromDown(cnt) - calPhi0;
        if (abs(dphi) > abs(calPeak)) calPeak = dphi;
        if (tc >= CAL_WIN) calPhase = 1;
      } else if (calPhase == 1) {               // freia
        vCmd = fmaxf(vCmd - CAL_ACC * DT, 0.0f);
        setArmVelocity(vCmd);
        if (vCmd <= 0.0f) { calResult(); calPhase = 2; testTimer = millis(); }
      } else if (calPhase == 2) {               // espera a haste balançar
        if (millis() - testTimer > 1500) calPhase = 3;
      } else {
        if (moveToward(0.0f)) gotoState(ST_IDLE);   // volta o braço ao zero
      }
      break;
    }

    case ST_RETURN:
      uCmd = 0;
      if (moveToward(0.0f)) gotoState(ST_IDLE);
      break;

    case ST_TEST:
      if (testPhase == 0) {
        if (moveToward(HALF_PI)) {
          Serial.printf("# braco em +90 graus (passos: %ld). Confira o angulo real!\n", (long)(g_stepPos - stepZero));
          testPhase = 1; testTimer = millis();
        }
      } else if (testPhase == 1) {
        if (millis() - testTimer > 1500) testPhase = 2;
      } else if (testPhase == 2) {
        if (moveToward(0.0f)) gotoState(ST_IDLE);
      } else if (testPhase >= 20) {             // comando n: teste de micropasso
        const int32_t N = 100;                  // 100 passos a 50 passos/s (devagar: nao perde passo)
        const float   V = 50.0f / STEPS_PER_RAD;
        int32_t moved = g_stepPos - testStart;
        if (testPhase == 20) {
          setArmVelocity(V);
          if (moved >= N) {
            setArmVelocity(0); testPhase = 21; testTimer = millis();
            Serial.println(F("\n=============== TESTE DE MICROPASSO (n) ==============="));
            Serial.printf("Enviados %ld passos ao driver. Quanto o braco girou DE VERDADE?\n", (long)moved);
            for (int ms = 16; ms >= 1; ms /= 2)
              Serial.printf("   ~%6.1f graus -> driver em 1/%-2d %s\n", moved * 360.0f / (MOTOR_STEPS_REV * ms), ms,
                            ms == MICROSTEPS ? "(o que o codigo espera: OK)" : "");
            Serial.println(F("Se nao for o esperado: ligue MS1, MS2, MS3 ao VDD (1/16) ou ajuste MICROSTEPS."));
            Serial.println(F("O braco volta em 3 s..."));
            Serial.println(F("=======================================================\n"));
          }
        } else if (testPhase == 21) {
          if (millis() - testTimer > 3000) testPhase = 22;
        } else {
          setArmVelocity(-V);
          if (moved <= 0) { setArmVelocity(0); vCmd = 0; gotoState(ST_IDLE); }
        }
      } else {                                   // testPhase 10: comando j (jog)
        if (moveToward(jogTarget)) {
          Serial.printf("# motor chegou em %+.2f graus (%+ld passos)\n",
                        alpha * RAD_TO_DEG, (long)(g_stepPos - stepZero));
          gotoState(ST_IDLE);
        }
      }
      break;
  }
}

// ============================================================================
//  SERIAL
// ============================================================================
// Leitura por linha, sem bloquear (Monitor Serial com "Nova linha" ou "Ambos NL e CR")
static void processCommand(const char* line) {
  char  c   = line[0];
  float arg = atof(line + 1);
  {
    switch (c) {
      case 'h': printHelp(); break;
      case 'm':
        outMode = OUT_MONITOR;
        if (arg >= 1 && arg <= 10) monPeriodMs = (uint32_t)(1000.0f / arg);
        Serial.printf("# painel legivel a %.1f Hz\n", 1000.0f / monPeriodMs);
        break;
      case 'p': outMode = OUT_PLOTTER; break;
      case 's': outMode = OUT_SILENT; Serial.println(F("# saida silenciada (m volta ao painel)")); break;
      case 'j': {
        if (state != ST_IDLE && state != ST_STOP) { Serial.println(F("# 'j' so com o equilibrio parado")); break; }
        if (arg == 0) { Serial.println(F("# use j<graus>, ex.: j30 ou j-30")); break; }
        motorEnable(true);
        jogTarget = constrain(alpha + arg * DEG_TO_RAD, -ARM_LIMIT, ARM_LIMIT);
        testPhase = 10; vCmd = 0; state = ST_TEST;
        Serial.printf("# motor: girando %+.1f graus (de %+.1f para %+.1f)\n",
                      arg, alpha * RAD_TO_DEG, jogTarget * RAD_TO_DEG);
        break;
      }
      case 'k': printModel(); break;
      case 'x': perActive = false; gotoState(ST_STOP); break;
      case 'e': motorEnable(true); if (state == ST_STOP) gotoState(ST_IDLE); break;
      case 'g':
        if (!motorEnabled) motorEnable(true);
        gotoState(ST_IDLE);
        Serial.println(F("# armado: levante a haste ate a vertical e solte devagar"));
        break;
      case 'z':
        if (state == ST_BALANCE) break;
        stepZero = g_stepPos; Serial.println(F("# zero do braco definido"));
        break;
      case 'c':
        if (state == ST_BALANCE) break;
        pcnt_unit_clear_count(s_pcnt); prevCount = 0; thetaOffset = 0;
        Serial.println(F("# encoder zerado (haste pendurada = 180 graus)"));
        break;
      case 't':
        if (state != ST_IDLE && state != ST_STOP) break;
        motorEnable(true);
        testPhase = 0; vCmd = 0; state = ST_TEST;
        Serial.println(F("# teste: braco vai a +90 graus e volta"));
        break;
      case 'n':
        if (state != ST_IDLE && state != ST_STOP) { Serial.println(F("# 'n' so com o equilibrio parado")); break; }
        motorEnable(true);
        testStart = g_stepPos; testPhase = 20; vCmd = 0; state = ST_TEST;
        Serial.println(F("# teste de micropasso: 100 passos devagar (2 s). Marque a posicao do braco e observe."));
        break;
      case 'v':
        if (state != ST_IDLE && state != ST_STOP) { Serial.println(F("# 'v' so com o equilibrio parado")); break; }
        if (fabsf(thetaFromDown) > 2.0f * DEG_TO_RAD || fabsf(thetaDot) > 0.2f) {
          Serial.println(F("# 'v': a haste precisa estar PENDURADA e PARADA")); break;
        }
        motorEnable(true);
        calPhase = 0; calPeak = 0; calPhi0 = countsFromDown(encoderRaw());
        calT0 = micros(); vCmd = 0; state = ST_CAL;
        Serial.println(F("# calibrando: o braco vai dar um tranco de ~30 graus e voltar..."));
        break;
      case 'w':
        if (state == ST_BALANCE) { Serial.println(F("# 'w' so com a haste pendurada")); break; }
        perActive = true; perArmed = false; perCount = 0; perAmp = 0; perStart = millis();
        perMax = perMin = 0; perHavePrevMin = false; perCenterSum = 0; perCenterN = 0;
        Serial.printf("# medindo periodo: de um empurraozinho na haste pendurada (< 15 graus); %d ciclos...\n", PER_CYCLES);
        break;
      case 'r': {
        float deg = arg;
        if (state != ST_BALANCE) { Serial.println(F("# 'r' so durante o equilibrio")); break; }
        setReference(deg);
        Serial.printf("# referencia do braco = %.1f graus\n", alphaRef * RAD_TO_DEG);
        break;
      }
      case 'q': {
        float deg = arg;
        if (state != ST_BALANCE) { Serial.println(F("# 'q' so durante o equilibrio")); break; }
        if (logIdx >= 0 || reportIdx >= 0) { Serial.println(F("# aguarde o ensaio/relatorio atual")); break; }
        if (fabsf(deg) < 5.0f) { Serial.println(F("# use um degrau de pelo menos 5 graus (ideal 60)")); break; }
        float from = alphaRef * RAD_TO_DEG;
        float to   = constrain(from + deg, -REF_MAX * RAD_TO_DEG, REF_MAX * RAD_TO_DEG);
        if (fabsf(to - from) < 5.0f) { Serial.println(F("# degrau fora do limite de +/-90 graus")); break; }
        const char* comma = strchr(line, ',');
        logRunsTotal = comma ? constrain(atoi(comma + 1), 1, 10) : 1;
        logStartDeg = from; logStepDeg = to - from;
        for (int i = 0; i < LOG_N; i++) logAcc[i] = 0;
        logRuns = 0;
        segFrom = from; segTo = to;
        logIdx = 0; logDiv = 0;
        setReference(to);
        Serial.printf("# ensaio: %d degrau(s) alternados entre %.1f e %.1f graus, %d s cada\n",
                      logRunsTotal, from, to, LOG_SECONDS);
        break;
      }
      default: break;
    }
  }
}

static void handleSerial() {
  static char buf[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n > 0) { buf[n] = '\0'; processCommand(buf); n = 0; }
    } else if (n < sizeof(buf) - 1 && ch != ' ') {
      buf[n++] = ch;
    }
  }
}

static void sendTelemetry() {                   // modo 'p' (Plotter Serial)
  Serial.printf("theta:%.2f,thetaDot:%.2f,alpha:%.2f,ref:%.2f,vArm:%.2f,u_x0.1:%.1f,trim:%.2f\n",
                theta * RAD_TO_DEG, thetaDot, alpha * RAD_TO_DEG, alphaRef * RAD_TO_DEG,
                vCmd, uCmd * 0.1f, thetaOffset * RAD_TO_DEG);
}

// ----------------------------------------------------------------------------
//  Painel legível (modo 'm', padrão) — para o Monitor Serial do Arduino IDE
// ----------------------------------------------------------------------------
static const char* stateText(State s) {
  switch (s) {
    case ST_STOP:    return "STOP - motor desligado (comando x)";
    case ST_IDLE:    return "IDLE - aguardando a haste na vertical";
    case ST_BALANCE: return "BALANCE - equilibrando";
    case ST_RETURN:  return "RETURN - braco voltando ao zero";
    case ST_TEST:    return "TEST - movendo o motor";
    case ST_CAL:     return "CAL - calibrando sentido e raio";
  }
  return "?";
}

static void printMonitor() {
  static uint32_t lastMs    = millis();
  static int32_t  lastCount = 0;
  static int32_t  lastSteps = 0;

  uint32_t now = millis();
  float    dt  = (now - lastMs) * 1e-3f;
  if (dt <= 0) return;

  // --- encoder ---
  int32_t cnt = encoderRaw();
  int32_t dc  = cnt - lastCount;                // o PCNT volta a 0 em ±ENC_LIMIT
  if (dc >  ENC_LIMIT / 2) dc -= ENC_LIMIT;
  if (dc < -ENC_LIMIT / 2) dc += ENC_LIMIT;
  float cntRate = dc / dt;

  uint32_t ta = encTogA, tb = encTogB;
  encTogA = encTogB = 0;
  if ((ta >= 4 && tb == 0) || (tb >= 4 && ta == 0)) encFault = true;   // só um canal mexe
  else if (ta > 0 && tb > 0) encFault = false;

  const char* encStatus;
  if (encFault)                   encStatus = (tb == 0) ? "FALHA: canal B parado (fio branco/pull-up)"
                                                        : "FALHA: canal A parado (fio verde/pull-up)";
  else if (ta > 0 && tb > 0)      encStatus = "OK, haste girando";
  else if (encSawA && encSawB)    encStatus = "OK, haste parada";
  else                            encStatus = "AGUARDANDO: gire a haste com a mao para testar";

  // --- motor ---
  int32_t steps    = g_stepPos - stepZero;
  int32_t dSteps   = g_stepPos - lastSteps;
  float   stepRate = dSteps / dt;
  float   armVel   = stepRate / STEPS_PER_RAD;
  float   rpm      = armVel * 60.0f / (2.0f * PI);

  const char* motStatus;
  if (!motorEnabled)               motStatus = "DESLIGADO (eixo livre)";
  else if (fabsf(stepRate) > 1.0f) motStatus = "LIGADO, girando";
  else                             motStatus = "LIGADO, parado (segurando posicao)";

  lastMs = now; lastCount = cnt; lastSteps = g_stepPos;

  Serial.printf("\n------------ t = %8.1f s | %s ------------\n", now * 1e-3f, stateText(state));

  Serial.printf("ENCODER : %s\n", encStatus);
  Serial.printf("          sinais A=%u B=%u | contagem = %+7ld | %+6.0f contagens/s\n",
                encLevelA, encLevelB, (long)cnt, cntRate);
  // Ângulo da haste em [0, 360): 0 = pendurada, 180 = em pé (nos dois sentidos de giro).
  // O "desvio da vertical" (o que o controle usa) só aparece perto de cima, onde faz sentido.
  float angDeg = fmodf(theta * RAD_TO_DEG + 180.0f, 360.0f);
  if (angDeg < 0) angDeg += 360.0f;
  if (angDeg >= 359.995f) angDeg = 0.0f;
  if (fabsf(theta) < HALF_PI) {
    Serial.printf("          haste = %6.2f graus (0 = pendurada, 180 = em pe) | desvio da vertical = %+6.2f graus | %+6.2f rad/s\n",
                  angDeg, theta * RAD_TO_DEG, thetaDot);
    if (state == ST_IDLE && fabsf(theta) < 20.0f * DEG_TO_RAD) {
      float xu = unstableMode() * RAD_TO_DEG;
      Serial.printf("CAPTURA : x = desvio + vel/w = %+5.1f graus (engata com |x| < %.0f) -> %s\n", xu,
                    ENGAGE_XU * RAD_TO_DEG,
                    fabsf(xu) < ENGAGE_XU * RAD_TO_DEG ? "DENTRO: pode soltar"
                    : "fora: aproxime mais devagar e pare perto de 180");
    }
  }
  else
    Serial.printf("          haste = %6.2f graus (0 = pendurada, 180 = em pe) | desvio da vertical =   ---  | %+6.2f rad/s\n",
                  angDeg, thetaDot);

  Serial.printf("MOTOR   : %s | EN=%s\n", motStatus,
                PIN_EN < 0 ? "fixo no GND" : (motorEnabled ? "0 (habilitado)" : "1 (desabilitado)"));
  Serial.printf("          braco = %+8.2f graus | %+7ld passos (1/%d) | %+6.2f rad/s = %+6.1f rpm | %+6.0f passos/s\n",
                alpha * RAD_TO_DEG, (long)steps, MICROSTEPS, armVel, rpm, stepRate);

  if (state == ST_BALANCE) {
    Serial.printf("CONTROLE: u = %+7.1f rad/s2 | ref = %+6.1f graus | integral -> %+5.1f rad/s2 (compensa zero ~%+.2f graus)\n",
                  uCmd, alphaRef * RAD_TO_DEG, -K5 * zInt, -K5 * zInt / K1 * RAD_TO_DEG);
  }
}

// ============================================================================
//  SETUP / LOOP
// ============================================================================
void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== Pendulo invertido rotacional - ESP32 ==="));

  pinMode(PIN_STEP, OUTPUT); digitalWrite(PIN_STEP, LOW);
  pinMode(PIN_DIR,  OUTPUT); digitalWrite(PIN_DIR, DIR_INVERT ? LOW : HIGH);
  if (PIN_EN >= 0) pinMode(PIN_EN, OUTPUT);
  motorEnable(false);

  logAcc = (float*)malloc(sizeof(float) * LOG_N);
  if (!encoderBegin() || !logAcc) {
    Serial.println(F("ERRO: falha ao iniciar o encoder (PCNT) ou a memoria do ensaio."));
    while (true) delay(1000);
  }
  Serial.println(F("Mantenha a haste PENDURADA e parada: ela define a referencia (180 graus)."));

  // Verificação inicial das ligações
  Serial.println(F("\n--- Verificacao inicial ---"));
  Serial.printf("Encoder: A no GPIO%d = %d | B no GPIO%d = %d | contagem = %ld\n",
                PIN_ENC_A, gpio_get_level((gpio_num_t)PIN_ENC_A),
                PIN_ENC_B, gpio_get_level((gpio_num_t)PIN_ENC_B), (long)encoderRaw());
  Serial.println(F("         -> gire a haste com a mao: o painel deve mostrar 'OK' e ~2400 contagens/volta"));
  Serial.printf("Motor  : STEP no GPIO%d | DIR no GPIO%d | EN %s | micropasso 1/%d (%d passos/volta)\n",
                PIN_STEP, PIN_DIR, PIN_EN < 0 ? "fixo no GND" : (String("no GPIO") + PIN_EN).c_str(),
                MICROSTEPS, MOTOR_STEPS_REV * MICROSTEPS);
  Serial.println(F("         -> envie t (ou j30) e confira se o braco gira o angulo mostrado"));
  Serial.println(F("Antes de equilibrar (haste PENDURADA):"));
  Serial.println(F("  1) envie w e de um empurrao de 10-15 graus -> mede o periodo e acha o ZERO verdadeiro"));
  Serial.println(F("  2) com a haste parada, envie v -> confere os sentidos e estima o raio do braco"));
  Serial.println(F("  3) leve a haste DEVAGAR ate ~180 graus, quase parada, e solte com cuidado (sem empurrar)"));
  encLevelA = gpio_get_level((gpio_num_t)PIN_ENC_A);
  encLevelB = gpio_get_level((gpio_num_t)PIN_ENC_B);

  computeGains(defaultA());
  printModel();
  printHelp();

  s_timer = timerBegin(1000000);
  timerAttachInterrupt(s_timer, &onStepTimer);
  timerAlarm(s_timer, 1000000 / STEP_ISR_HZ, true, 0);

  motorEnable(true);
  stepZero = g_stepPos;
  gotoState(ST_IDLE);
  Serial.println(F("# pronto: levante a haste ate a vertical para iniciar o equilibrio"));
}

void loop() {
  static uint32_t nextCtrl  = micros();
  static uint32_t lastTelem = 0;

  uint32_t now = micros();
  if ((int32_t)(now - nextCtrl) >= 0) {
    nextCtrl += (uint32_t)(1e6f / CTRL_HZ);
    if ((int32_t)(now - nextCtrl) > 5000) nextCtrl = now;
    controlStep();
  }

  handleSerial();

  // Fim do ensaio de degrau → resumo + CSV enviado aos poucos (não bloqueia o controle)
  if (logIdx >= LOG_N) {
    logRuns++;
    if (logRuns < logRunsTotal) {               // próximo degrau, no sentido oposto
      float f = segFrom; segFrom = segTo; segTo = f;
      logIdx = 0; logDiv = 0;
      setReference(segTo);
      Serial.printf("# degrau %d/%d: %.1f -> %.1f graus\n", logRuns + 1, logRunsTotal, segFrom, segTo);
    } else {
      logIdx = -1;
      reportStepTest();
    }
  }
  reportStream();

  // Saída periódica (pausada durante o ensaio de degrau e o envio do CSV)
  if (logIdx < 0 && reportIdx < 0) {
    if (outMode == OUT_PLOTTER && millis() - lastTelem >= 20) {            // 50 Hz
      lastTelem = millis();
      sendTelemetry();
    } else if (outMode == OUT_MONITOR && millis() - lastTelem >= monPeriodMs &&
               Serial.availableForWrite() > 700) {                        // nunca bloqueia
      lastTelem = millis();
      printMonitor();
    }
  }
}
