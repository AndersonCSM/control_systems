# Controle de Pêndulo Invertido Rotativo com ESP32 — PID via Lugar Geométrico das Raízes (LGR)

## Abstract

**Rotary Inverted Pendulum Control on ESP32 using Root-Locus PID Design**

This repository presents the complete mathematical modeling, Root-Locus (LGR) PID controller synthesis, and real-time embedded C/Arduino implementation for stabilizing a rotary inverted pendulum at its unstable upright equilibrium ($180^\circ$) using an **ESP32** microcontroller, a **NEMA 14 bipolar stepper motor** driven by an **A4988** microstepping driver ($1/16$ step, $3200\text{ steps/rev}$), and a **600 PPR incremental optical rotary encoder** ($2400\text{ CPR}$ in $4\times$ quadrature, $0.15^\circ/\text{pulse}$).

To overcome the practical challenges of stepper-driven inverted pendulums—such as electromagnetic interference (EMI) on encoder lines, derivative quantization spikes at $1\text{ kHz}$ sampling, and non-minimum phase arm drift—the firmware implements:
1. **A deterministic $20\text{ kHz}$ ($50\text{ }\mu\text{s}$) Hardware Timer ISR in `IRAM`** that simultaneously samples the quadrature encoder via direct register reads (`GPIO.in1.val`) with a state-transition lookup table (eliminating GPIO interrupt floods), synthesizes jitter-free `STEP` pulses via a 32-bit Direct Digital Synthesis (DDS) phase accumulator (`REG_WRITE`), and clocks the discrete PID control loop at an exact $1\text{ kHz}$ ($\Delta t = 1\text{ ms}$) rate.
2. **Two progressive firmware versions** for academic and practical comparison:
   - [`firmware/simples/`](firmware/simples/): Direct implementation of the theoretical Root-Locus PID ($UP = 5\%$, $T_s = 0.5\text{ s}$) and linearized plant transfer function $G(s) = \frac{1398.89}{s^2 - 33.8276}$.
   - [`firmware/melhorado/`](firmware/melhorado/): Bench-validated final controller incorporating a $15\text{ Hz}$ first-order low-pass derivative filter (attenuating $0.15^\circ$ quantization noise without compromising phase margin), a leaky integrator ($\text{leak}_i = 0.995$) for smooth manual swing-up capture, and soft arm position/velocity feedback to keep the motor centered around $0^\circ$.

---

## Estrutura do Repositório

```text
repositorio/
├── README.md                          # Visão geral do projeto (PT-BR / EN), hardware e pinagem
├── docs/
│   ├── projeto.md                     # Memorial de cálculo completo em LaTeX (Modelagem, LGR e Ganhos)
│   ├── code.md                        # Documentação da arquitetura de tempo real, ISR de 20 kHz e filtros
│   ├── complementar.ipynb             # Cálculo automático de K e z + Tabela e Gráfico de validação na simulação
│   ├── img/                           # Figuras e gráficos de simulação
│   └── datasheets/                    # Datasheets do motor NEMA 14, driver A4988 e encoder incremental
└── firmware/
    ├── simples/                       # Versão 1: Implementação direta da teoria pura (PID + FT pura)
    │   ├── simples.ino
    │   ├── pid.h
    │   └── pid.c
    └── melhorado/                     # Versão 2 (Final): Com filtro derivativo (15 Hz), limite 4 kHz e controle do braço
        ├── melhorado.ino
        ├── pid.h
        └── pid.c
```

---

## Hardware e Pinagem (ESP32 DevKit 30 Pinos)

| Componente | Especificação | Pino ESP32 | Observação |
| :--- | :--- | :---: | :--- |
| **Encoder Incremental (Canal A)** | `E38S6G5-600B-G24N` ($600\text{ PPR} \implies 2400\text{ CPR}$ em $4\times$) | `GPIO 32` | Fio Verde (pull-up p/ $3,3\text{ V}$) |
| **Encoder Incremental (Canal B)** | Resolução de $0,15^\circ/\text{pulso}$ | `GPIO 33` | Fio Branco (pull-up p/ $3,3\text{ V}$) |
| **Driver Motor de Passo (`STEP`)** | `A4988` em $1/16$ microstepping ($3200\text{ passos/volta}$) | `GPIO 25` | Gerado via DDS a $20\text{ kHz}$ |
| **Driver Motor de Passo (`DIR`)** | Motor Bipolar NEMA 14 (`17PM-K044U`) | `GPIO 26` | Controle de sentido |
| **Driver Motor de Passo (`ENABLE`)**| Habilitação das bobinas | `GPIO 27` | Ativo em nível baixo (`LOW`) |

---

## Resumo dos Parâmetros de Projeto ([`docs/projeto.md`](docs/projeto.md))

* **Planta Linearizada:**
  $$G(s) = \frac{1398,89}{s^2 - 33,8276} \quad \left(p_{1,2} = \pm 5,8161\text{ rad/s}\right)$$
* **Critérios de Desempenho:** $UP = 5\%$ ($0,05$) e $T_s(2\%) = 0,5\text{ s}$ $\implies$ $\zeta = 0,6901$, $\omega_n = 11,5924\text{ rad/s}$, $s_d = -8,0000 \pm j8,3895$.
* **Controlador PID Projetado via LGR:**
  $$G_c(s) = \frac{K(s + z)^2}{s} = K_p + \frac{K_i}{s} + K_d s$$
  * $z = 5,9888\text{ rad/s}$
  * $K = K_d = 0,015601\text{ N}\cdot\text{m}\cdot\text{s/rad}$
  * $K_p = 2Kz = 0,186868\text{ N}\cdot\text{m/rad}$
  * $K_i = Kz^2 = 0,559556\text{ N}\cdot\text{m/(rad}\cdot\text{s)}$

---

## Como Executar na Bancada

1. Abra [`firmware/melhorado/melhorado.ino`](firmware/melhorado/melhorado.ino) na Arduino IDE e grave no ESP32.
2. Com a haste parada na posição vertical inferior ($0^\circ$), abra o Monitor Serial (`115200` baud) e envie `'c'` caso queira recalibrar o zero.
3. Levante a haste manualmente até próximo ao topo ($180^\circ \pm 30^\circ$): o controle armará automaticamente (`ctrl=ON`).
