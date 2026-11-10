# Materiais de Pesquisa, Referência e Propostas Futuras

Este diretório contém códigos de referência, versões preliminares e estudos analíticos mantidos como material de apoio para desenvolvedores e pesquisadores que forem dar continuidade ao projeto do pêndulo invertido.

> [!NOTE]
> O firmware de produção validado na bancada física está localizado em [`firmware/final/`](../firmware/final/).

---

## Estrutura do Diretório

```text
materiais_pesquisa/
├── README.md                      # Este guia explicativo
├── ref/                           # Códigos de referência da bancada do laboratório e estudos iniciais
│   ├── pendulo_invertido_esp32_final.ino  # Implementação acadêmica de referência com observador de estados e auto-trim
│   └── simples/                   # Implementação inicial com PID puro e FT linearizada
│       ├── simples.ino
│       ├── pid.c
│       └── pid.h
└── proposta_futura/               # Proposta de aprimoramentos e testes futuros
    ├── proposta_futura.ino
    ├── pid.c
    └── pid.h
```

### 1. `ref/` (Códigos de Referência)
- **`pendulo_invertido_esp32_final.ino`**: Código desenvolvido no laboratório contendo controle em cascata (haste PD + braço PID via LGR), observador de estados de 2ª ordem para interpolação de passos do encoder, auto-trim de zero e rotinas de autodiagnóstico via comandos seriais (`v`, `w`, `j`, `n`).
- **`simples/`**: Primeira versão do controlador PID baseada na modelagem linear direta $G(s)$, útil para comparação de desempenho entre controle clássico sem filtros e as versões com condicionamento de sinal.

### 2. `proposta_futura/` (Proposta de Evolução)
- Contém experimentações com desacoplamento acelerométrico direto, rotinas de calibração automática de centro de massa e estratégias de rejeição de perturbação dinâmica para continuidade acadêmica.
