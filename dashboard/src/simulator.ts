import { DEFAULT_LGR, gainsFromKz } from './protocol';

const PI = Math.PI;
const DEG2RAD = PI / 180.0;
const RAD2DEG = 180.0 / PI;

const DT = 0.001; // 1 kHz malha interna idêntica ao ESP32
const DERIV_FILT_HZ = 15.0; // 15 Hz validado em bancada no esp32_pid_final
const ALPHA_D = 1.0 - Math.exp(-2.0 * PI * DERIV_FILT_HZ * DT);

const M_MASSA = 0.0085;
const L_COMP = 0.29;
const G_ACCEL = 9.81;
const J_INERCIA = M_MASSA * L_COMP * L_COMP; // 0.00071485 kg.m^2
const DAMPING_B = 0.00035; // Amortecimento viscoso leve do mancal do encoder

const PASSOS_POR_VOLTA = 3200.0;
const FREQ_MAX_HZ = 4000.0; // 4000 Hz validado em bancada no esp32_pid_final
const CPR = 2400;

const K_POS_BRACO = 0.0025;
const K_VEL_BRACO = 0.0012;
const LIMITE_BRACO_DEG = 130.0;

export type SimulatorFirmwareMode = 'melhorado' | 'simples';

/**
 * Simulador físico não-linear do pêndulo invertido + controlador PID idêntico ao C do ESP32.
 * Suporta emular tanto o firmware 'melhorado' quanto o 'simples'.
 */
export class PendulumSimulator {
  private readonly onLine: (line: string) => void;
  private timerId: ReturnType<typeof setInterval> | null = null;
  public firmwareMode: SimulatorFirmwareMode = 'melhorado';

  private tMs = 0;
  private thetaRad = (180.0 - 12.0) * DEG2RAD; // Inicia a 168° (dentro da zona ativa p/ estabilizar logo)
  private omegaRadS = 0.0;
  private thetaOffsetRad = 0.0;

  private bracoRad = 0.0;
  private velCmdRadS = 0.0;
  private controleAtivo = false;
  private sinalMotor = -1;
  private plantaSinal = -1; // Sentido físico verdadeiro da planta simulada (-1 = igual ao real calibrado)

  private kAtivo: number = DEFAULT_LGR.k;
  private zAtivo: number = DEFAULT_LGR.z;
  private modoKz = true;
  private kp = 0;
  private ki = 0;
  private kd = 0;

  private integral = 0;
  private erroAnterior = 0;
  private derivFiltrada = 0;

  private telemDiv = 0;

  constructor(onLine: (line: string) => void) {
    this.onLine = onLine;
    const g = gainsFromKz(this.kAtivo, this.zAtivo);
    this.kp = g.kp;
    this.ki = g.ki;
    this.kd = g.kd;
  }

  get isRunning(): boolean {
    return this.timerId !== null;
  }

  setFirmwareMode(mode: SimulatorFirmwareMode): void {
    this.firmwareMode = mode;
    this.resetPid();
    const modeName = mode === 'simples' ? 'SIMPLES (PID Puro)' : 'MELHORADO (DDS + Filtro 15Hz)';
    this.onLine(`E,SIM_MODE,${modeName}`);
  }

  toggleFirmwareMode(): SimulatorFirmwareMode {
    const next = this.firmwareMode === 'melhorado' ? 'simples' : 'melhorado';
    this.setFirmwareMode(next);
    return next;
  }

  start(): void {
    if (this.timerId !== null) return;
    this.tMs = 0;
    this.thetaRad = 165.0 * DEG2RAD;
    this.omegaRadS = 2.85; // Velocidade típica ao levantar a haste entrando na zona de captura
    this.bracoRad = 0;
    this.velCmdRadS = 0;
    this.controleAtivo = false;
    this.resetPid();

    this.onLine('==================================================');
    if (this.firmwareMode === 'simples') {
      this.onLine('BOOT OK - Controle 2 SIMPLES (PID Puro + FT Pura)');
    } else {
      this.onLine('BOOT OK - Controle 2 FINAL SLIM (Deriv=15Hz | FreqMax=4000Hz)');
    }
    this.onLine('==================================================');
    this.emitParams();
    this.onLine('E,BOOT,SIMULADOR');

    // Executa 10 passos de 1 ms a cada 10 ms (tempo real 100 Hz de telemetria)
    this.timerId = setInterval(() => {
      for (let i = 0; i < 10; i++) {
        this.step1ms();
      }
    }, 10);
  }

  stop(): void {
    if (this.timerId !== null) {
      clearInterval(this.timerId);
      this.timerId = null;
    }
  }

  /** Levanta manualmente a haste até perto de 180° (ex.: após uma queda) */
  liftToTop(initialOffsetDeg = -8.0): void {
    this.thetaRad = (180.0 + initialOffsetDeg) * DEG2RAD + this.thetaOffsetRad;
    this.omegaRadS = 0.0;
    this.bracoRad = 0.0;
    this.velCmdRadS = 0.0;
    this.onLine(`E,SIM_LIFT,Haste posicionada em ${(180.0 + initialOffsetDeg).toFixed(1)} deg`);
  }

  /** Aplica um peteleco (impulso de velocidade angular / deslocamento) na haste */
  perturb(deltaDeg = 7.5): void {
    this.thetaRad += deltaDeg * DEG2RAD;
    this.omegaRadS += deltaDeg * 0.18 * DEG2RAD * 10;
    this.onLine(`E,SIM_PERTURB,Perturbacao aplicada (${deltaDeg >= 0 ? '+' : ''}${deltaDeg.toFixed(1)} deg)`);
  }

  /** Avança N milissegundos de forma síncrona (útil para testes automatizados) */
  advanceMs(ms: number): void {
    for (let i = 0; i < ms; i++) {
      this.step1ms();
    }
  }

  sendCommand(raw: string): void {
    const trimmed = raw.trim();
    if (!trimmed) return;

    if (trimmed.length === 1) {
      const ch = trimmed.toLowerCase();
      if (ch === 'c') {
        this.thetaOffsetRad = this.thetaRad;
        this.onLine('E,ZERO');
        return;
      }
      if (ch === 'i') {
        this.sinalMotor = -this.sinalMotor;
        this.onLine(`E,INVERT,${this.sinalMotor > 0 ? '+1' : '-1'}`);
        this.emitParams();
        return;
      }
      if (ch === 'a') {
        this.controleAtivo = false;
        this.velCmdRadS = 0;
        this.sinalMotor = this.plantaSinal;
        this.thetaRad = 0;
        this.omegaRadS = 0;
        this.thetaOffsetRad = 0;
        this.bracoRad = 0;
        this.onLine('E,AUTOCAL_START');
        this.onLine(`E,AUTOCAL_DONE,${this.sinalMotor > 0 ? '+1' : '-1'}`);
        this.emitParams();
        return;
      }
      if (ch === 'p') {
        this.emitParams();
        return;
      }
      if (ch === 'l') {
        this.onLine('================ INICIO LOG.TXT (SIMULADOR) ================');
        this.onLine(`# Modo=${this.modoKz ? 'KZ' : 'PID'} | Kp=${this.kp.toFixed(6)} | Ki=${this.ki.toFixed(6)} | Kd=${this.kd.toFixed(6)}`);
        this.onLine('================ FIM LOG.TXT ================');
        return;
      }
    }

    if (trimmed.startsWith('k=') || trimmed.startsWith('K=')) {
      const parts = trimmed.slice(2).split(',');
      if (parts.length !== 2) {
        this.onLine('E,ERR,Formato esperado: k=<K>,<z>');
        return;
      }
      const nk = Number(parts[0]);
      const nz = Number(parts[1]);
      if (!Number.isFinite(nk) || !Number.isFinite(nz) || nk <= 0 || nk > 5 || nz < 0 || nz > 200) {
        this.onLine('E,ERR,Valores fora da faixa segura (0 < K <= 5, 0 <= z <= 200)');
        return;
      }
      this.kAtivo = nk;
      this.zAtivo = nz;
      this.modoKz = true;
      const g = gainsFromKz(nk, nz);
      this.applyGainsBumpless(g.kp, g.ki, g.kd);
      this.onLine(`E,GAINS_KZ,${nk.toFixed(6)},${nz.toFixed(4)}`);
      this.emitParams();
      return;
    }

    if (trimmed.startsWith('g=') || trimmed.startsWith('G=')) {
      const parts = trimmed.slice(2).split(',');
      if (parts.length !== 3) {
        this.onLine('E,ERR,Formato esperado: g=<Kp>,<Ki>,<Kd>');
        return;
      }
      const nkp = Number(parts[0]);
      const nki = Number(parts[1]);
      const nkd = Number(parts[2]);
      if (
        !Number.isFinite(nkp) ||
        !Number.isFinite(nki) ||
        !Number.isFinite(nkd) ||
        nkp < 0 ||
        nkp > 50 ||
        nki < 0 ||
        nki > 500 ||
        nkd < 0 ||
        nkd > 5 ||
        (nkp === 0 && nki === 0 && nkd === 0)
      ) {
        this.onLine('E,ERR,Ganhos fora da faixa segura (Kp<=50, Ki<=500, Kd<=5)');
        return;
      }
      this.modoKz = false;
      this.applyGainsBumpless(nkp, nki, nkd);
      this.onLine(`E,GAINS_PID,${this.kp.toFixed(6)},${this.ki.toFixed(6)},${this.kd.toFixed(6)}`);
      this.emitParams();
      return;
    }

    this.onLine(`E,ERR,Comando desconhecido: ${trimmed}`);
  }

  private resetPid(): void {
    this.integral = 0;
    this.erroAnterior = 0;
    this.derivFiltrada = 0;
  }

  private applyGainsBumpless(nkp: number, nki: number, nkd: number): void {
    if (this.controleAtivo) {
      if (nki > 1e-6 && this.ki > 1e-6) {
        this.integral *= this.ki / nki;
        const maxInt = DEFAULT_LGR.torqueMaxNm / nki;
        this.integral = Math.max(-maxInt, Math.min(maxInt, this.integral));
      } else if (nki <= 1e-6) {
        this.integral = 0;
      }
    } else {
      this.resetPid();
    }
    this.kp = nkp;
    this.ki = nki;
    this.kd = nkd;
  }

  private emitParams(): void {
    const sinStr = this.sinalMotor >= 0 ? `+${this.sinalMotor}` : `${this.sinalMotor}`;
    this.onLine(
      `P,${this.kp.toFixed(6)},${this.ki.toFixed(6)},${this.kd.toFixed(6)},${this.kAtivo.toFixed(6)},${this.zAtivo.toFixed(4)},${DEFAULT_LGR.setpointDeg.toFixed(1)},${DEFAULT_LGR.zonaDeg.toFixed(1)},${DEFAULT_LGR.torqueMaxNm.toFixed(3)},${sinStr},${DEFAULT_LGR.telemHz},${this.modoKz ? 1 : 0}`,
    );
  }

  private encoderReadDeg(): number {
    const relRad = this.thetaRad - this.thetaOffsetRad;
    const counts = Math.round((relRad / (2 * PI)) * CPR);
    let mod = counts % CPR;
    if (mod < 0) mod += CPR;
    return (mod * 360.0) / CPR;
  }

  private calcularPid(setpointDeg: number, angDeg: number): number {
    const e = (setpointDeg - angDeg) * DEG2RAD;
    const pTerm = this.kp * e;
    const derivBruta = (e - this.erroAnterior) / DT;

    let dTerm = 0;
    let iNovo = 0;

    if (this.firmwareMode === 'simples') {
      // Firmware Simples: Derivada pura sem filtro e integrador puro sem vazamento (leak)
      dTerm = this.kd * derivBruta;
      iNovo = this.integral + e * DT;
    } else {
      // Firmware Melhorado: Filtro passa-baixas de 15 Hz na derivada e leak_i = 0.995
      this.derivFiltrada += ALPHA_D * (derivBruta - this.derivFiltrada);
      dTerm = this.kd * this.derivFiltrada;
      iNovo = this.integral * 0.995 + e * DT;
    }

    const iTerm = this.ki * iNovo;

    let u = pTerm + iTerm + dTerm;
    const lim = DEFAULT_LGR.torqueMaxNm;
    if (u > lim) {
      u = lim;
      if (e < 0) this.integral = iNovo;
    } else if (u < -lim) {
      u = -lim;
      if (e > 0) this.integral = iNovo;
    } else {
      this.integral = iNovo;
    }

    this.erroAnterior = e;
    return u;
  }

  private step1ms(): void {
    this.tMs += 1;

    const ang = this.encoderReadDeg();
    let bracoDeg = this.bracoRad * RAD2DEG;
    const r = DEFAULT_LGR.setpointDeg;
    let tauPid = 0;
    let tau = 0;

    const dentroTopo = Math.abs(r - ang) <= DEFAULT_LGR.zonaDeg;
    const caboSeguro = this.firmwareMode === 'simples' || !this.controleAtivo || Math.abs(bracoDeg) <= LIMITE_BRACO_DEG;

    if (dentroTopo && caboSeguro) {
      if (!this.controleAtivo) {
        this.resetPid();
        this.erroAnterior = (r - ang) * DEG2RAD;
        this.velCmdRadS = 0;
        this.bracoRad = 0;
        bracoDeg = 0;
        this.controleAtivo = true;
        this.onLine('E,CTRL_ON');
      }

      tauPid = this.calcularPid(r, ang);
      const tauBraco = this.firmwareMode === 'simples'
        ? 0
        : -K_POS_BRACO * this.bracoRad - K_VEL_BRACO * this.velCmdRadS;
      tau = (tauPid + tauBraco) * this.sinalMotor;

      const tauClamped = Math.max(-DEFAULT_LGR.torqueMaxNm, Math.min(DEFAULT_LGR.torqueMaxNm, tau));
      const alphaCmd = tauClamped / J_INERCIA;
      this.velCmdRadS += alphaCmd * DT;

      let freqSigned = this.velCmdRadS * (PASSOS_POR_VOLTA / (2 * PI));
      if (freqSigned > FREQ_MAX_HZ) {
        freqSigned = FREQ_MAX_HZ;
        this.velCmdRadS = freqSigned * ((2 * PI) / PASSOS_POR_VOLTA);
      } else if (freqSigned < -FREQ_MAX_HZ) {
        freqSigned = -FREQ_MAX_HZ;
        this.velCmdRadS = freqSigned * ((2 * PI) / PASSOS_POR_VOLTA);
      }
      this.bracoRad += this.velCmdRadS * DT;
    } else {
      if (this.controleAtivo) {
        const motivo = !dentroTopo
          ? 'Haste saiu da zona do topo (±30 deg)'
          : 'Braco atingiu limite do cabo (±130 deg)';
        this.controleAtivo = false;
        this.velCmdRadS = 0;
        this.resetPid();
        this.onLine(`E,CTRL_OFF,${motivo}`);
      }
    }

    // Dinâmica não-linear da haste: 0 rad = pendurado p/ baixo (estável), pi rad = topo (instável)
    // Se sinalMotor == plantaSinal (-1 * -1 = +1), a reação do braço estabiliza a haste.
    const tauReacaoHaste = this.controleAtivo ? tau * this.plantaSinal : 0;
    const gravTorque = -M_MASSA * G_ACCEL * L_COMP * Math.sin(this.thetaRad);
    const dampTorque = -DAMPING_B * this.omegaRadS;
    const alphaHaste = (gravTorque + dampTorque + tauReacaoHaste) / J_INERCIA;

    this.omegaRadS += alphaHaste * DT;
    this.thetaRad += this.omegaRadS * DT;

    this.telemDiv++;
    if (this.telemDiv >= 10) {
      this.telemDiv = 0;
      const angOut = this.encoderReadDeg();
      const bracoOut = this.bracoRad * RAD2DEG;
      const freqStep = this.velCmdRadS * (PASSOS_POR_VOLTA / (2 * PI));
      this.onLine(
        `T,${this.tMs},${this.controleAtivo ? 1 : 0},${angOut.toFixed(2)},${(r - angOut).toFixed(2)},${tauPid.toFixed(4)},${tau.toFixed(4)},${freqStep.toFixed(1)},${bracoOut.toFixed(1)}`,
      );
    }
  }
}
