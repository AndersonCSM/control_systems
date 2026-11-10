# Controle de Pêndulo Invertido Rotativo com ESP32 — PID via Lugar Geométrico das Raízes (LGR)

## Abstract

**Rotary Inverted Pendulum Control on ESP32 using Root-Locus PID Design**

This repository presents the complete mathematical modeling, Root-Locus (LGR) PID controller synthesis, and real-time embedded C/Arduino implementation for stabilizing a rotary inverted pendulum at its unstable upright equilibrium ($180^\circ$) using an **ESP32** microcontroller, a **NEMA 14 bipolar stepper motor** driven by an **A4988** microstepping driver ($1/16$ step, $3200\text{ steps/rev}$), and a **600 PPR incremental optical rotary encoder** ($2400\text{ CPR}$ in $4\times$ quadrature, $0.15^\circ/\text{pulse}$).

To overcome the practical challenges of stepper-driven inverted pendulums—such as electromagnetic interference (EMI) on encoder lines, derivative quantization spikes at $1\text{ kHz}$ sampling, and non-minimum phase arm drift—the firmware implements:
1. **A deterministic $20\text{ kHz}$ ($50\text{ }\mu\text{s}$) Hardware Timer ISR in `IRAM`** that simultaneously samples the quadrature encoder via direct register reads (`GPIO.in1.val`) with a state-transition lookup table (eliminating GPIO interrupt floods), synthesizes jitter-free `STEP` pulses via a 32-bit Direct Digital Synthesis (DDS) phase accumulator (`REG_WRITE`), and clocks the discrete PID control loop at an exact $1\text{ kHz}$ ($\Delta t = 1\text{ ms}$) rate.
2. **Production-Ready Firmware ([`firmware/final/`](firmware/final/)):** Bench-validated controller combining LGR gains ($K = 0.015601$, $z = 5.9888\text{ rad/s}$), a $15\text{ Hz}$ first-order low-pass derivative filter, smart 3-zone integrator with anti-windup, soft arm position/velocity feedback, and safety cable protection ($\pm 130^\circ$).
3. **Research and Reference Materials ([`materiais_pesquisa/`](materiais_pesquisa/)):** Academic reference implementations and future proposals for advanced state-space and observers.

---

## Bancada Experimental

| Pêndulo Equilibrado no Topo ($180^\circ$) | Bancada Experimental Completa | Eletrônica (ESP32 + Shield A4988) |
| :---: | :---: | :---: |
| ![Pêndulo Equilibrado](docs/img/pendulo_equilibrado.jpeg) | ![Bancada Experimental](docs/img/bancada.jpeg) | ![ESP32 e Shield](docs/img/esp32_shield.jpeg) |

---

## Estrutura do Repositório

```text
control_systems/
├── README.md                          # Visão geral do projeto, fotos da bancada, hardware e guia de execução
├── docs/
│   ├── projeto.md                     # Memorial de cálculo completo em LaTeX (Modelagem, LGR e Ganhos)
│   ├── code.md                        # Documentação da arquitetura de tempo real, ISR de 20 kHz e filtros
│   ├── complementar.ipynb             # Cálculo automático de K e z + Validação da simulação
│   ├── img/                           # Fotos da bancada física e esquemáticos
│   │   ├── pendulo_equilibrado.jpeg   # Haste em equilíbrio vertical superior estável
│   │   ├── bancada.jpeg               # Visão geral da bancada experimental
│   │   └── esp32_shield.jpeg          # Microcontrolador ESP32 e Shield A4988
│   └── datasheets/                    # Datasheets do motor NEMA 14, driver A4988 e encoder incremental
│
├── firmware/                          # FIRMWARE FINAL DE PRODUÇÃO (Gravar na bancada)
│   └── final/                         # Diretório compatível com Arduino IDE
│       ├── final.ino                  # Código-fonte principal com temporizador de 20 kHz e controle a 1 kHz
│       ├── pid.c                      # Implementação do PID com filtro derivativo e anti-windup
│       └── pid.h                      # Estruturas e protótipos do controlador
│
├── materiais_pesquisa/                # MATERIAL DE APOIO, PESQUISA E CONTINUIDADE
│   ├── README.md                      # Descrição dos materiais de apoio
│   ├── ref/                           # Códigos de referência de laboratório e estudos iniciais
│   │   ├── pendulo_invertido_esp32_final.ino  # Implementação acadêmica com observador de estados
│   │   └── simples/                   # Versão inicial com FT pura sem filtros
│   │       ├── simples.ino
│   │       ├── pid.c
│   │       └── pid.h
│   └── proposta_futura/               # Propostas de melhorias analíticas futuras
│       ├── proposta_futura.ino
│       ├── pid.c
│       └── pid.h
│
└── dashboard/                         # Interface de telemetria e gráficos em tempo real
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

1. Abra a pasta [`firmware/final/final.ino`](firmware/final/final.ino) na Arduino IDE (ou PlatformIO) e grave no ESP32.
2. Com a haste parada na posição vertical inferior ($0^\circ$), abra o Monitor Serial (`115200` baud).
3. Se necessário, envie `'c'` para calibrar a referência com a haste em repouso apontando para baixo ($0,00^\circ$).
4. Levante a haste manualmente até próximo ao topo ($180^\circ \pm 30^\circ$): o controle armará automaticamente (`ctrl=ON`).
5. Caso o braço gire mais de $\pm 130^\circ$, o desarme de segurança desliga o motor para proteger o cabo do encoder.

---

## Pontos de Atenção, Limitações Físicas e Oportunidades de Melhoria

Durante os ensaios práticos na bancada experimental e o desenvolvimento iterativo do controle, foram identificadas restrições físicas e comportamentos dinâmicos que exigem atenção dos operadores e servem como oportunidades para trabalhos futuros:

### 1. Estabilidade Vinculada a um Ponto Fixo do Eixo
* **Comportamento Atual:** No firmware de produção ([`firmware/final/final.ino`](firmware/final/final.ino)), a coordenada de centro do braço (`step_zero`) é congelada na posição exata em que o motor se encontra no milissegundo de acionamento do controle no topo. A malha do braço atua como uma mola virtual tentando mantê-lo rigidamente ancorado a essa coordenada angular.
* **Limitação:** O sistema só sustenta o equilíbrio se puder retornar a esse ponto fixo específico do motor. Ele não permite que o operador desloque o pêndulo suavemente pela bancada ou altere a referência espacial de operação em tempo real sem gerar esforço restaurador.
* **Trabalho Futuro:** Implementação de seguimento de referência externa com pré-filtro dinâmico (feedforward) ou desacoplamento entre equilíbrio e posicionamento livre.

### 2. Limitação Física do Cabeamento e Restrição de Giro ($\pm 130^\circ$)
* **Restrição Mecânica:** O encoder óptico incremental de $600\text{ PPR}$ está montado no eixo móvel da haste, e sua fiação conecta-se diretamente à shield do ESP32 na base fixa. Como a montagem **não possui anel coletor elétrico (*slip ring*)**, o motor não pode girar continuamente sem torcer, tracionar e eventualmente romper os fios dos canais A/B e alimentação.
* **Proteção no Firmware:** Para blindar o hardware, implementou-se o desligamento instantâneo do motor caso a excursão ultrapasse `LIMITE_BRACO_DEG = 130.0f`.
* **Impacto no Controle:** Essa restrição reduz a bacia de atração do sistema: distúrbios maiores que exijam mais de $130^\circ$ de deslocamento para recuperar o pêndulo desarmam o motor por segurança antes que o equilíbrio possa ser restaurado.

### 3. Sensibilidade a Distúrbios e Giro Excessivo do Braço
* **Comportamento:** Ao aplicar pequenos toques manuais na haste, o motor reage rapidamente com pulsos de aceleração para colocar a base sob o centro de gravidade em queda.
* **Causa Identificada:** Embora o controlador da haste restabeleça a vertical rapidamente, o braço do motor acumula velocidade angular. Como o amortecimento viscoso virtual (`K_VEL_BRACO`) e o retorno elástico (`K_POS_BRACO`) são calibrados para serem suaves e não derrubarem a haste, o braço leva um longo percurso angular para frear totalmente, gerando excursões amplas que se aproximam do limite do cabo.
* **Tentativas Exploradas:** O aumento excessivo do amortecimento no modelo de torque gerou instabilidade na haste. Modelos em espaço de estados com desacoplamento por aceleração direta foram experimentados e encontram-se documentados em [`materiais_pesquisa/`](materiais_pesquisa/).

### 4. Deriva Direcional por Erro Estático de Zero do Encoder
* **Desafio:** Se a calibração com o comando `'c'` for executada com a haste minimamente fora da vertical inferior real (ex.: $0,2^\circ$ de desvio residual provocado por atrito do rolamento ou desnível da bancada), a posição de referência de $180^\circ$ não coincidirá perfeitamente com o topo gravitacional verdadeiro.
* **Consequência:** A haste em $180,0^\circ$ sofre uma força de tombamento constante devida à gravidade. Como a malha integra o torque em aceleração do motor, o braço passa a se mover continuamente em uma única direção (*creep/drift* contínuo), enrolando o cabo até desarmar.
* **Tentativas Exploradas:** No diretório [`materiais_pesquisa/ref/`](materiais_pesquisa/ref/), preservou-se uma rotina de autocalibração dinâmica baseada no centro de oscilação do pêndulo em balanço livre (comando `'w'`), que calcula o repouso verdadeiro compensando o decaimento por atrito.

### 5. Dinâmica de Fase Não-Mínima do Pêndulo Furuta
* **Característica Físico-Matemática:** A planta linearizada do braço rotativo possui um zero de transmissão no semiplano direito (RHP). Isso significa que, para deslocar a base em um sentido, o motor precisa primeiramente acelerar ligeiramente no **sentido contrário** para tombar a haste antes de avançar. Esse efeito físico impõe um limite fundamental de largura de banda e exige cautela na sintonia dos ganhos do braço para evitar oscilações de baixa frequência (*limit cycle*).

