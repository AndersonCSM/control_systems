# Pêndulo Invertido · Dashboard Desktop (`.exe` & `.AppImage`)

Aplicativo desktop local (Windows `.exe` e Linux `.AppImage`) para visualização em tempo real da **resposta no tempo** do pêndulo invertido (`Setpoint 180°` × `Ângulo θ`), indicadores de desempenho transitório e regime permanente (`Tempo equilibrado`, `Erro RMS`, `Sobressinal`, `Acomodação ±5°`) e sintonia ao vivo do controlador PID por **dois zeros reais e iguais** $C(s) = \frac{K(s+z)^2}{s}$ (`Kp = 2Kz`, `Ki = Kz²`, `Kd = K`) ou `Kp, Ki, Kd` diretos.

---

## Executáveis gerados (`dashboard/release/`)

- **Linux (`.AppImage`):** `release/Pendulo-PID-Dashboard-1.0.0-x86_64.AppImage`
  ```bash
  chmod +x release/Pendulo-PID-Dashboard-1.0.0-x86_64.AppImage
  ./release/Pendulo-PID-Dashboard-1.0.0-x86_64.AppImage
  ```
- **Windows (`.exe` portátil, sem instalador):** `release/Pendulo-PID-Dashboard-1.0.0-win-x64.exe`
  - Basta executar diretamente no Windows 10/11 (64 bits).

### Regerar os executáveis

```bash
cd dashboard
npm install
npm run dist:all     # Gera simultaneamente o .AppImage (Linux) e o .exe (Windows)
# ou individualmente:
npm run dist:linux
npm run dist:win
```

Para abrir em modo desenvolvimento local:
```bash
npm run electron:start   # Abre a janela nativa Electron localmente
# ou no navegador:
npm run dev
```

---

## Permissões para atuar no ESP32

1. **Permissões internas do aplicativo:** o processo principal do Electron (`electron/main.cjs`) habilita automaticamente as políticas de acesso serial (`setPermissionCheckHandler` e `setDevicePermissionHandler`) e apresenta um seletor nativo das portas conectadas (`COMx` no Windows; `/dev/ttyUSB*` e `/dev/ttyACM*` no Linux), destacando chips USB do ESP32 (`CP2102`, `CH340/CH9102`, `FTDI`, `Espressif USB-CDC`).
2. **Windows (`.exe`):** portas `COMx` já possuem permissão de leitura/escrita para o usuário local sem necessidade de administrador.
3. **Linux (`.AppImage`):**
   - Caso a porta `/dev/ttyUSB0` ou `/dev/ttyACM0` esteja sem permissão de escrita (`EACCES`), ao selecioná-la (ou clicando em **Mais ações ▾ → Configurar permissão USB (Linux)**) o aplicativo solicita autorização gráfica padrão do sistema (`pkexec`) para instalar `/etc/udev/rules.d/99-esp32-pendulo.rules`, recarregar o `udev` e liberar a porta **imediatamente**, sem precisar reiniciar a sessão.
   - Se preferir configurar manualmente via terminal: `sudo usermod -aG dialout $USER`.

---

## Firmware do ESP32 (`esp32_pid_final.ino`)

- **Taxa serial padrão:** `115200 baud` (também selecionável `921600` para alta taxa) · Telemetria em tempo real · Malha interna PID a `1 kHz` · Gerador STEP/Encoder a `20 kHz`.
- **Compatibilidade do Dashboard:**
  - Suporta telemetria textual de bancada do `esp32_pid_final`:
    `ctrl=ON | pulsos=1200 | ang=180.05° | braco=+0.2° | tau=+0.0123 N.m | step=+150.0 Hz`
  - Suporta telemetria CSV de alta taxa (`T,...` a 100 Hz).
  - Reconhece eventos de zeramento pelo botão BOOT da placa ESP32 (`GPIO 0`).
  - Ganhos padrão calibrados do LGR: $K = 0.01851$, $z = 6.3640$ ($K_p = 0.2356$, $K_i = 0.7497$, $K_d = 0.0185$).
  - Limite de frequência STEP validado na bancada: $4000\text{ Hz}$.
  - Filtro derivativo passa-baixas: $15\text{ Hz}$.
- **Pinagem padrão (ESP32 DevKit 30 pinos):**
  - `ENC_A=32`, `ENC_B=33`, `STEP=25`, `DIR=26`, `EN=27`, `BOOT=0`
- **Comandos disponíveis (em `Mais ações ▾` e na barra inferior):**
  - `c` — zerar encoder com a haste embaixo (0.00°)
  - `k=<K>,<z>` — aplica $K$ e $z$ sem tranco derivativo/integral (*bumpless transfer*)
  - `g=<Kp>,<Ki>,<Kd>` — aplica $K_p, K_i, K_d$ diretos
  - `a` — auto-calibrar sentido e zerar encoder em `0°`
  - `i` — inverter motor
  - `p` — sincronizar parâmetros com a placa
  - `l` — despejar `LOG.TXT` de alta resolução
