import './style.css';
import { TelemetryBuffer } from './buffer';
import { TimeResponseChart } from './charts';
import {
  DEFAULT_LGR,
  buildKzCommand,
  buildPidCommand,
  gainsFromKz,
  parseLine,
  type ControllerParams,
  type TelemetrySample,
} from './protocol';
import { SerialConnection, isWebSerialSupported } from './serial';
import { PendulumSimulator } from './simulator';
import { StabilityDetector, type StabilityReport } from './stability';

interface DesktopSerialPortInfo {
  portId: string;
  portName: string;
  devPath: string;
  displayName: string;
  vendorId: string;
  productId: string;
  isEsp32: boolean;
  hasPermission: boolean;
}

interface DesktopAPI {
  isDesktop: boolean;
  platform: string;
  onChooseSerialPort: (cb: (ports: DesktopSerialPortInfo[]) => void) => () => void;
  confirmSerialPort: (sel: {
    portId: string;
    devPath?: string;
    needsPermFix?: boolean;
  }) => Promise<{ ok: boolean; message?: string; cancelled?: boolean }>;
  setupLinuxUsbPermissions: () => Promise<{ ok: boolean; message: string }>;
}

declare global {
  interface Window {
    desktopAPI?: DesktopAPI;
  }
}

function mustEl<T extends HTMLElement>(id: string): T {
  const el = document.getElementById(id);
  if (!el) throw new Error(`Elemento #${id} não encontrado`);
  return el as T;
}

function fmtComma(val: number, decimals: number): string {
  return val.toFixed(decimals).replace('.', ',');
}

function parseDecimal(raw: string): number {
  return Number(raw.trim().replace(',', '.'));
}

// Elementos do cabeçalho e menu
const subtitleInfo = mustEl<HTMLParagraphElement>('subtitle-info');
const statusPill = mustEl<HTMLDivElement>('status-pill');
const statusText = mustEl<HTMLSpanElement>('status-text');
const btnConnect = mustEl<HTMLButtonElement>('btn-connect');
const btnSim = mustEl<HTMLButtonElement>('btn-sim');
const btnMore = mustEl<HTMLButtonElement>('btn-more');
const moreMenu = mustEl<HTMLDivElement>('more-menu');
const simMenuGroup = mustEl<HTMLDivElement>('sim-menu-group');
const baudSelect = mustEl<HTMLSelectElement>('baud-select');
const menuSinal = mustEl<HTMLSpanElement>('menu-sinal');

const cmdAutocal = mustEl<HTMLButtonElement>('cmd-autocal');
const cmdInvert = mustEl<HTMLButtonElement>('cmd-invert');
const cmdLog = mustEl<HTMLButtonElement>('cmd-log');
const btnSimLift = mustEl<HTMLButtonElement>('btn-sim-lift');
const btnSimPerturb = mustEl<HTMLButtonElement>('btn-sim-perturb');
const btnOpenPidModal = mustEl<HTMLButtonElement>('btn-open-pid-modal');
const btnOpenConsole = mustEl<HTMLButtonElement>('btn-open-console');
const btnCsv = mustEl<HTMLButtonElement>('btn-csv');
const btnClear = mustEl<HTMLButtonElement>('btn-clear');
const btnLinuxPerm = mustEl<HTMLButtonElement>('btn-linux-perm');

// KPIs
const kpiTime = mustEl<HTMLElement>('kpi-time');
const kpiRms = mustEl<HTMLElement>('kpi-rms');
const kpiOvershoot = mustEl<HTMLElement>('kpi-overshoot');
const kpiSettling = mustEl<HTMLElement>('kpi-settling');

// Área do gráfico
const fallChip = mustEl<HTMLSpanElement>('fall-chip');
const windowSelect = mustEl<HTMLSelectElement>('window-select');
const btnPause = mustEl<HTMLButtonElement>('btn-pause');

// Barra inferior de sintonia K e z
const formKz = mustEl<HTMLFormElement>('form-kz');
const inpK = mustEl<HTMLInputElement>('inp-k');
const inpZ = mustEl<HTMLInputElement>('inp-z');
const btnResetLgr = mustEl<HTMLButtonElement>('btn-reset-lgr');
const readoutKp = mustEl<HTMLElement>('readout-kp');
const readoutKi = mustEl<HTMLElement>('readout-ki');
const readoutKd = mustEl<HTMLElement>('readout-kd');

// Modais
const portModal = mustEl<HTMLDivElement>('port-modal');
const portListEl = mustEl<HTMLDivElement>('port-list');
const btnClosePortModal = mustEl<HTMLButtonElement>('btn-close-port-modal');
const btnCancelPort = mustEl<HTMLButtonElement>('btn-cancel-port');

const pidModal = mustEl<HTMLDivElement>('pid-modal');
const formPidDirect = mustEl<HTMLFormElement>('form-pid-direct');
const inpKp = mustEl<HTMLInputElement>('inp-kp');
const inpKi = mustEl<HTMLInputElement>('inp-ki');
const inpKd = mustEl<HTMLInputElement>('inp-kd');
const btnClosePidModal = mustEl<HTMLButtonElement>('btn-close-pid-modal');
const btnCancelPid = mustEl<HTMLButtonElement>('btn-cancel-pid');

const consoleModal = mustEl<HTMLDivElement>('console-modal');
const consoleOutput = mustEl<HTMLPreElement>('console-output');
const corruptCounter = mustEl<HTMLSpanElement>('corrupt-counter');
const btnCloseConsoleModal = mustEl<HTMLButtonElement>('btn-close-console-modal');
const btnCopyConsole = mustEl<HTMLButtonElement>('btn-copy-console');
const btnClearConsole = mustEl<HTMLButtonElement>('btn-clear-console');

const toastEl = mustEl<HTMLDivElement>('toast');

// Instâncias principais
const buffer = new TelemetryBuffer(2400, 60_000);
const stability = new StabilityDetector();
const chart = new TimeResponseChart(mustEl<HTMLElement>('chart-response'));

let setpointDeg: number = DEFAULT_LGR.setpointDeg;
let zonaDeg: number = DEFAULT_LGR.zonaDeg;
let paused = false;
let windowSec = Number(windowSelect.value) || 10;
let dirtyChart = false;
let latestSample: TelemetrySample | null = null;
let latestStability: StabilityReport | null = null;
let corruptLines = 0;

let toastTimer: ReturnType<typeof setTimeout> | null = null;
function showToast(msg: string, durationMs = 3200): void {
  toastEl.textContent = msg;
  toastEl.classList.remove('hidden');
  if (toastTimer) clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    toastEl.classList.add('hidden');
  }, durationMs);
}

const MAX_CONSOLE_LINES = 600;
const consoleLines: string[] = [];
function appendConsole(text: string): void {
  const ts = new Date().toLocaleTimeString('pt-BR', { hour12: false });
  consoleLines.push(`[${ts}] ${text}`);
  if (consoleLines.length > MAX_CONSOLE_LINES) {
    consoleLines.splice(0, consoleLines.length - MAX_CONSOLE_LINES);
  }
  consoleOutput.textContent = consoleLines.join('\n');
  consoleOutput.scrollTop = consoleOutput.scrollHeight;
}

// Integração Desktop (.AppImage / .exe)
if (window.desktopAPI?.platform === 'linux') {
  btnLinuxPerm.classList.remove('hidden');
}

if (window.desktopAPI) {
  window.desktopAPI.onChooseSerialPort((ports) => {
    renderPortPickerModal(ports);
  });
}

function renderPortPickerModal(ports: DesktopSerialPortInfo[]): void {
  portListEl.innerHTML = '';

  if (ports.length === 0) {
    const empty = document.createElement('p');
    empty.className = 'modal-desc';
    empty.textContent =
      'Nenhuma porta serial USB detectada. Verifique o cabo USB do ESP32 (alimentação + dados) e tente novamente.';
    portListEl.appendChild(empty);
  } else {
    for (const p of ports) {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'port-option';

      const left = document.createElement('div');
      left.className = 'port-main';
      const strong = document.createElement('strong');
      strong.textContent = p.devPath;
      const small = document.createElement('small');
      const vidPid = p.vendorId ? ` · VID:PID ${p.vendorId}:${p.productId}` : '';
      small.textContent = `${p.displayName || 'Porta Serial'}${vidPid}`;
      left.append(strong, small);

      const right = document.createElement('div');
      right.className = 'port-badges';
      if (p.isEsp32) {
        const b = document.createElement('span');
        b.className = 'badge-esp';
        b.textContent = 'ESP32 / USB';
        right.appendChild(b);
      }
      if (!p.hasPermission) {
        const perm = document.createElement('span');
        perm.className = 'badge-perm';
        perm.textContent = 'Requer permissão';
        right.appendChild(perm);
      }

      btn.append(left, right);
      btn.addEventListener('click', async () => {
        portModal.classList.add('hidden');
        const res = await window.desktopAPI?.confirmSerialPort({
          portId: p.portId,
          devPath: p.devPath,
          needsPermFix: !p.hasPermission,
        });
        if (res && !res.ok && !res.cancelled && res.message) {
          showToast(res.message, 4500);
          appendConsole(`[PERMISSÃO USB] ${res.message}`);
        }
      });

      portListEl.appendChild(btn);
    }
  }

  portModal.classList.remove('hidden');
}

function cancelPortPicker(): void {
  if (!portModal.classList.contains('hidden')) {
    portModal.classList.add('hidden');
    void window.desktopAPI?.confirmSerialPort({ portId: '' });
  }
}

btnClosePortModal.addEventListener('click', cancelPortPicker);
btnCancelPort.addEventListener('click', cancelPortPicker);

// Conexão Serial & Simulador
const serial = new SerialConnection({
  onLine: handleIncomingLine,
  onStatusChange: (connected, msg) => {
    btnConnect.dataset.active = String(connected);
    btnConnect.textContent = connected ? 'Desconectar USB' : 'Conectar USB';
    refreshStatusHeader();
    if (msg) appendConsole(`[USB] ${msg}`);
  },
  onError: (err) => {
    appendConsole(`[ERRO USB] ${err}`);
    showToast(err, 4200);
  },
});

const simulator = new PendulumSimulator(handleIncomingLine);

function refreshStatusHeader(): void {
  const baud = baudSelect.value;
  subtitleInfo.textContent = `ESP32 PID Final · Setpoint ${Math.round(setpointDeg)}° (±${Math.round(zonaDeg)}°) · ${baud} baud`;

  if (!serial.isConnected && !simulator.isRunning) {
    statusPill.dataset.state = 'disconnected';
    statusText.textContent = 'Desconectado';
    return;
  }

  if (!latestStability) {
    statusPill.dataset.state = 'connected';
    statusText.textContent = simulator.isRunning ? 'Simulação ativa' : 'USB conectado';
    return;
  }

  switch (latestStability.state) {
    case 'ESTAVEL':
      statusPill.dataset.state = 'stable';
      statusText.textContent = 'Estável · 180°';
      break;
    case 'ESTABILIZANDO':
      statusPill.dataset.state = 'settling';
      statusText.textContent = 'Estabilizando';
      break;
    case 'CAIU':
      statusPill.dataset.state = 'fallen';
      statusText.textContent = 'Caiu / Desarmado';
      break;
    case 'DESARMADO':
    default:
      statusPill.dataset.state = 'connected';
      statusText.textContent = simulator.isRunning ? 'Simulação · Aguardando 180°' : 'Aguardando 180°';
      break;
  }
}

async function sendToTarget(cmd: string): Promise<void> {
  if (serial.isConnected) {
    await serial.send(cmd);
    return;
  }
  if (simulator.isRunning) {
    simulator.sendCommand(cmd);
    return;
  }
  showToast('Conecte ao ESP32 via USB ou clique em "Simular planta".');
}

function updateGainReadouts(kp: number, ki: number, kd: number): void {
  readoutKp.textContent = fmtComma(kp, 4);
  readoutKi.textContent = fmtComma(ki, 4);
  readoutKd.textContent = fmtComma(kd, 4);
}

function applyParamsToUi(p: ControllerParams): void {
  setpointDeg = p.setpointDeg;
  zonaDeg = p.zonaDeg;
  chart.updateConfig(p.setpointDeg, p.zonaDeg);

  updateGainReadouts(p.kp, p.ki, p.kd);
  menuSinal.textContent = p.sinalMotor >= 0 ? `+${p.sinalMotor}` : `${p.sinalMotor}`;

  if (document.activeElement !== inpK) inpK.value = fmtComma(p.k, 6);
  if (document.activeElement !== inpZ) inpZ.value = fmtComma(p.z, 4);
  if (document.activeElement !== inpKp) inpKp.value = fmtComma(p.kp, 6);
  if (document.activeElement !== inpKi) inpKi.value = fmtComma(p.ki, 6);
  if (document.activeElement !== inpKd) inpKd.value = fmtComma(p.kd, 6);
  refreshStatusHeader();
}

function handleIncomingLine(rawLine: string): void {
  const msg = parseLine(rawLine);
  if (!msg) return;

  switch (msg.type) {
    case 'T': {
      latestSample = msg.sample;
      latestStability = stability.update(msg.sample, setpointDeg);
      buffer.push(msg.sample, setpointDeg);
      dirtyChart = true;
      break;
    }
    case 'P': {
      applyParamsToUi(msg.params);
      appendConsole(
        `[PARAM] Modo=${msg.params.modoKz ? 'KZ' : 'PID'} | Kp=${msg.params.kp.toFixed(6)} Ki=${msg.params.ki.toFixed(6)} Kd=${msg.params.kd.toFixed(6)} | K=${msg.params.k.toFixed(6)} z=${msg.params.z.toFixed(4)}`,
      );
      break;
    }
    case 'E': {
      if (msg.event.code === 'CTRL_OFF' && msg.event.detail) {
        stability.notifyFallReason(msg.event.detail);
      }
      if (msg.event.code === 'AUTOCAL_DONE') {
        const sinal = msg.event.detail ? ` (${msg.event.detail})` : '';
        showToast(`Sentido calibrado${sinal} e encoder zerado em 0°!`);
      }
      if (msg.event.code === 'ZERO') {
        showToast('Encoder zerado em 0°');
      }
      if (msg.event.code === 'ERR') {
        showToast(msg.event.detail || 'Comando inválido');
      }
      const suffix = msg.event.detail ? ` — ${msg.event.detail}` : '';
      appendConsole(`[EVENTO] ${msg.event.code}${suffix}`);
      break;
    }
    case 'corrupt': {
      corruptLines++;
      corruptCounter.textContent = `${corruptLines} ${corruptLines === 1 ? 'linha descartada' : 'linhas descartadas'}`;
      break;
    }
    case 'raw': {
      if (/^\d+,[-+]?\d/.test(msg.text)) {
        consoleLines.push(msg.text);
        if (consoleLines.length > MAX_CONSOLE_LINES) {
          consoleLines.splice(0, consoleLines.length - MAX_CONSOLE_LINES);
        }
        consoleOutput.textContent = consoleLines.join('\n');
      } else {
        appendConsole(msg.text);
      }
      break;
    }
  }
}

function renderMetrics(sample: TelemetrySample, rep: StabilityReport): void {
  refreshStatusHeader();

  if (sample.ctrl) {
    kpiTime.textContent = `${fmtComma(rep.activeDurationSec, 1)} s`;
    kpiRms.textContent = `${fmtComma(rep.rmsErroDeg, 1)}°`;
    kpiOvershoot.textContent = `${fmtComma(rep.overshootPct, 1)}%`;
    kpiSettling.textContent = rep.settlingTimeSec !== null ? `${fmtComma(rep.settlingTimeSec, 1)} s` : '…';
    fallChip.classList.add('hidden');
  } else {
    kpiTime.textContent = rep.bestDurationSec > 0 ? `0,0 s` : '-- s';
    kpiRms.textContent = '--°';
    kpiOvershoot.textContent = '--%';
    kpiSettling.textContent = '-- s';
    if (rep.state === 'CAIU' && rep.fallReason) {
      fallChip.textContent = rep.fallReason;
      fallChip.classList.remove('hidden');
    } else {
      fallChip.classList.add('hidden');
    }
  }
}

function animationLoop(): void {
  if (dirtyChart && !paused) {
    dirtyChart = false;
    chart.render(buffer.snapshot(windowSec));
    if (latestSample && latestStability) {
      renderMetrics(latestSample, latestStability);
    }
  }
  requestAnimationFrame(animationLoop);
}
requestAnimationFrame(animationLoop);

// Botões principais
btnConnect.addEventListener('click', async () => {
  closeMoreMenu();
  if (serial.isConnected) {
    await serial.disconnect();
    return;
  }
  if (!isWebSerialSupported()) {
    showToast('Use o aplicativo Desktop (.exe / .AppImage) ou Chrome/Edge para conectar via USB.', 4500);
    return;
  }
  if (simulator.isRunning) {
    stopSimulation();
  }
  buffer.clear();
  stability.reset();
  const baud = Number(baudSelect.value) || 115200;
  await serial.connect(baud);
});

function startSimulation(): void {
  buffer.clear();
  stability.reset();
  latestSample = null;
  latestStability = null;
  simulator.start();
  btnSim.dataset.active = 'true';
  btnSim.textContent = 'Parar simulação';
  simMenuGroup.classList.remove('hidden');
  refreshStatusHeader();
}

function stopSimulation(): void {
  simulator.stop();
  btnSim.dataset.active = 'false';
  btnSim.textContent = 'Simular planta';
  simMenuGroup.classList.add('hidden');
  latestStability = null;
  refreshStatusHeader();
}

btnSim.addEventListener('click', async () => {
  closeMoreMenu();
  if (simulator.isRunning) {
    stopSimulation();
    return;
  }
  if (serial.isConnected) {
    await serial.disconnect();
  }
  startSimulation();
});

// Dropdown "Mais ações"
function closeMoreMenu(): void {
  moreMenu.classList.add('hidden');
  btnMore.setAttribute('aria-expanded', 'false');
}

btnMore.addEventListener('click', (ev) => {
  ev.stopPropagation();
  const willOpen = moreMenu.classList.contains('hidden');
  moreMenu.classList.toggle('hidden', !willOpen);
  btnMore.setAttribute('aria-expanded', String(willOpen));
});

document.addEventListener('click', (ev) => {
  if (!moreMenu.classList.contains('hidden') && !moreMenu.contains(ev.target as Node) && ev.target !== btnMore) {
    closeMoreMenu();
  }
});

document.addEventListener('keydown', (ev) => {
  if (ev.key === 'Escape') {
    closeMoreMenu();
    cancelPortPicker();
    pidModal.classList.add('hidden');
    consoleModal.classList.add('hidden');
    return;
  }

  const target = ev.target as HTMLElement | null;
  const tag = target?.tagName?.toLowerCase();
  if (tag === 'input' || tag === 'textarea' || tag === 'select') {
    return;
  }

  if (ev.key === 'a' || ev.key === 'A') {
    cmdAutocal.click();
  } else if (ev.key === 'c' || ev.key === 'C') {
    void sendToTarget('c\n');
    showToast('Encoder zerado em 0°');
  } else if (ev.key === 'i' || ev.key === 'I') {
    cmdInvert.click();
  } else if (ev.key === 'l' || ev.key === 'L') {
    cmdLog.click();
  } else if (ev.key === ' ' || ev.code === 'Space') {
    if (tag !== 'button') {
      ev.preventDefault();
      btnPause.click();
    }
  }
});

baudSelect.addEventListener('change', () => {
  refreshStatusHeader();
});

// Comandos do menu
cmdAutocal.addEventListener('click', () => {
  closeMoreMenu();
  void sendToTarget('a\n');
  showToast('Iniciando calibração de sentido e zeramento do encoder…');
});

cmdInvert.addEventListener('click', () => {
  closeMoreMenu();
  void sendToTarget('i\n');
});

cmdLog.addEventListener('click', () => {
  closeMoreMenu();
  consoleModal.classList.remove('hidden');
  void sendToTarget('l\n');
});

btnSimLift.addEventListener('click', () => {
  closeMoreMenu();
  simulator.liftToTop(-15.0);
});

btnSimPerturb.addEventListener('click', () => {
  closeMoreMenu();
  simulator.perturb(8.0);
});

btnOpenPidModal.addEventListener('click', () => {
  closeMoreMenu();
  pidModal.classList.remove('hidden');
});

btnOpenConsole.addEventListener('click', () => {
  closeMoreMenu();
  consoleModal.classList.remove('hidden');
});

btnLinuxPerm.addEventListener('click', async () => {
  closeMoreMenu();
  if (!window.desktopAPI) return;
  const res = await window.desktopAPI.setupLinuxUsbPermissions();
  showToast(res.message, 4500);
  appendConsole(`[PERMISSÃO USB] ${res.message}`);
});

// Janela, Pausa, Limpar e CSV
windowSelect.addEventListener('change', () => {
  windowSec = Number(windowSelect.value) || 10;
  chart.setWindowSec(windowSec);
  dirtyChart = true;
  if (paused) {
    chart.render(buffer.snapshot(windowSec));
  }
});

btnPause.addEventListener('click', () => {
  paused = !paused;
  btnPause.textContent = paused ? 'Retomar' : 'Pausar';
  btnPause.dataset.active = String(paused);
  if (!paused) dirtyChart = true;
});

btnClear.addEventListener('click', () => {
  closeMoreMenu();
  buffer.clear();
  stability.reset();
  chart.render(buffer.snapshot(windowSec));
  kpiTime.textContent = '-- s';
  kpiRms.textContent = '--°';
  kpiOvershoot.textContent = '--%';
  kpiSettling.textContent = '-- s';
  fallChip.classList.add('hidden');
});

btnCsv.addEventListener('click', () => {
  closeMoreMenu();
  if (buffer.totalRecorded === 0) {
    showToast('Nenhuma amostra gravada ainda.');
    return;
  }
  const blob = new Blob([buffer.toCsv()], { type: 'text/csv;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  a.href = url;
  a.download = `pendulo_resposta_tempo_${stamp}.csv`;
  a.click();
  URL.revokeObjectURL(url);
  showToast(`CSV exportado (${buffer.totalRecorded} amostras).`);
});

// Sintonia inferior K e z (2 zeros reais iguais)
function previewKzReadout(): void {
  const k = parseDecimal(inpK.value);
  const z = parseDecimal(inpZ.value);
  if (Number.isFinite(k) && Number.isFinite(z) && k > 0 && z >= 0) {
    const g = gainsFromKz(k, z);
    updateGainReadouts(g.kp, g.ki, g.kd);
  }
}

inpK.addEventListener('input', previewKzReadout);
inpZ.addEventListener('input', previewKzReadout);
previewKzReadout();

btnResetLgr.addEventListener('click', () => {
  inpK.value = fmtComma(DEFAULT_LGR.k, 6);
  inpZ.value = fmtComma(DEFAULT_LGR.z, 4);
  previewKzReadout();
  if (serial.isConnected || simulator.isRunning) {
    const cmd = buildKzCommand(DEFAULT_LGR.k, DEFAULT_LGR.z);
    void sendToTarget(cmd);
    showToast('Padrão LGR restaurado e enviado.');
  }
});

formKz.addEventListener('submit', async (ev) => {
  ev.preventDefault();
  try {
    const k = parseDecimal(inpK.value);
    const z = parseDecimal(inpZ.value);
    const cmd = buildKzCommand(k, z);
    await sendToTarget(cmd);
    if (serial.isConnected || simulator.isRunning) {
      showToast(`Ganhos K=${fmtComma(k, 4)}, z=${fmtComma(z, 2)} aplicados.`);
    }
  } catch (err) {
    showToast(err instanceof Error ? err.message : String(err), 4000);
  }
});

// Modal Kp, Ki, Kd direto
btnClosePidModal.addEventListener('click', () => pidModal.classList.add('hidden'));
btnCancelPid.addEventListener('click', () => pidModal.classList.add('hidden'));

formPidDirect.addEventListener('submit', async (ev) => {
  ev.preventDefault();
  try {
    const kp = parseDecimal(inpKp.value);
    const ki = parseDecimal(inpKi.value);
    const kd = parseDecimal(inpKd.value);
    const cmd = buildPidCommand(kp, ki, kd);
    await sendToTarget(cmd);
    updateGainReadouts(kp, ki, kd);
    pidModal.classList.add('hidden');
    if (serial.isConnected || simulator.isRunning) {
      showToast('Ganhos Kp, Ki, Kd aplicados.');
    }
  } catch (err) {
    showToast(err instanceof Error ? err.message : String(err), 4000);
  }
});

// Modal Console
btnCloseConsoleModal.addEventListener('click', () => consoleModal.classList.add('hidden'));
btnClearConsole.addEventListener('click', () => {
  consoleLines.length = 0;
  consoleOutput.textContent = '';
});
btnCopyConsole.addEventListener('click', async () => {
  try {
    await navigator.clipboard.writeText(consoleLines.join('\n'));
    showToast('Console copiado para a área de transferência.');
  } catch {
    showToast('Não foi possível copiar automaticamente.');
  }
});

// Auto-start opcional via query ?sim=1
if (new URLSearchParams(window.location.search).get('sim') === '1') {
  startSimulation();
} else {
  refreshStatusHeader();
}
