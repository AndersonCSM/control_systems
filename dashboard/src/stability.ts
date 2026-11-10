import type { TelemetrySample } from './protocol';

export type StabilityState = 'DESARMADO' | 'ESTABILIZANDO' | 'ESTAVEL' | 'CAIU';

export interface StabilityThresholds {
  /** Janela de análise para critérios de estabilidade (ms) */
  windowMs: number;
  /** Erro RMS máximo permitido na janela (graus) */
  maxRmsErroDeg: number;
  /** Erro absoluto máximo permitido na janela (graus) */
  maxPeakErroDeg: number;
  /** Banda de acomodação em torno do setpoint (graus, ex.: ±5°) */
  settlingBandDeg: number;
  /** Excursão máxima do braço para considerar sem deriva (graus) */
  maxBracoDeg: number;
}

export const DEFAULT_STABILITY_THRESHOLDS: StabilityThresholds = {
  windowMs: 2000,
  maxRmsErroDeg: 2.0,
  maxPeakErroDeg: 5.0,
  settlingBandDeg: 5.0,
  maxBracoDeg: 90.0,
};

export interface StabilityReport {
  state: StabilityState;
  rmsErroDeg: number;
  maxAbsErroDeg: number;
  peakTorqueNm: number;
  activeDurationSec: number;
  bestDurationSec: number;
  /** Sobressinal percentual em relação ao setpoint (ex.: (186.9 - 180)/180 * 100 = 3.8%) */
  overshootPct: number;
  /** Tempo de acomodação na faixa de ±5° desde o acionamento do controle (null enquanto fora da faixa) */
  settlingTimeSec: number | null;
  /** Ponto de pico de excursão durante a captura atual */
  peakPoint: { tMs: number; angDeg: number } | null;
  fallReason: string | null;
}

export class StabilityDetector {
  private readonly thresholds: StabilityThresholds;
  private window: TelemetrySample[] = [];
  private wasCtrlActive = false;
  private ctrlStartMs: number | null = null;
  private entrySign = -1; // -1 se entrou abaixo de 180°, +1 se entrou acima de 180°
  private maxOvershootDeg = 0;
  private peakPoint: { tMs: number; angDeg: number } | null = null;
  private currentlyInsideBand = false;
  private settledAtMs: number | null = null;

  private activeDurationSec = 0;
  private bestDurationSec = 0;
  private fallen = false;
  private fallReason: string | null = null;

  constructor(thresholds: StabilityThresholds = DEFAULT_STABILITY_THRESHOLDS) {
    this.thresholds = thresholds;
  }

  notifyFallReason(reason: string): void {
    this.fallen = true;
    this.fallReason = reason;
  }

  reset(): void {
    this.window = [];
    this.wasCtrlActive = false;
    this.ctrlStartMs = null;
    this.maxOvershootDeg = 0;
    this.peakPoint = null;
    this.currentlyInsideBand = false;
    this.settledAtMs = null;
    this.activeDurationSec = 0;
    this.bestDurationSec = 0;
    this.fallen = false;
    this.fallReason = null;
  }

  update(sample: TelemetrySample, setpointDeg = 180.0): StabilityReport {
    if (sample.ctrl) {
      if (!this.wasCtrlActive || this.ctrlStartMs === null || sample.tMs < this.ctrlStartMs) {
        this.ctrlStartMs = sample.tMs;
        this.window = [];
        this.entrySign = sample.angDeg <= setpointDeg ? -1 : 1;
        this.maxOvershootDeg = 0;
        this.peakPoint = null;
        this.currentlyInsideBand = Math.abs(sample.angDeg - setpointDeg) <= this.thresholds.settlingBandDeg;
        this.settledAtMs = this.currentlyInsideBand ? sample.tMs : null;
        this.fallen = false;
        this.fallReason = null;
      }
      this.wasCtrlActive = true;

      // Sobressinal: excursão além do setpoint no sentido oposto ao de entrada
      const signedDelta = sample.angDeg - setpointDeg;
      const overshootCandidate = this.entrySign < 0 ? signedDelta : -signedDelta;
      if (overshootCandidate > this.maxOvershootDeg) {
        this.maxOvershootDeg = overshootCandidate;
        if (overshootCandidate >= 0.4) {
          this.peakPoint = { tMs: sample.tMs, angDeg: sample.angDeg };
        }
      }

      // Tempo de acomodação (±5°): instante em que entrou e permaneceu na banda
      const insideBand = Math.abs(signedDelta) <= this.thresholds.settlingBandDeg;
      if (!insideBand) {
        this.currentlyInsideBand = false;
        this.settledAtMs = null;
      } else if (!this.currentlyInsideBand) {
        this.currentlyInsideBand = true;
        this.settledAtMs = sample.tMs;
      }

      this.window.push(sample);
      const cutoff = sample.tMs - this.thresholds.windowMs;
      while (this.window.length > 1 && this.window[0].tMs < cutoff) {
        this.window.shift();
      }

      this.activeDurationSec = Math.max(0, (sample.tMs - this.ctrlStartMs) / 1000.0);
      if (this.activeDurationSec > this.bestDurationSec) {
        this.bestDurationSec = this.activeDurationSec;
      }

      let sumSqErro = 0;
      let maxAbsErro = 0;
      let peakTorque = 0;
      let maxAbsBraco = 0;

      for (const s of this.window) {
        const absErr = Math.abs(s.erroDeg);
        const absTau = Math.abs(s.tauTotalNm);
        const absBraco = Math.abs(s.bracoDeg);
        sumSqErro += s.erroDeg * s.erroDeg;
        if (absErr > maxAbsErro) maxAbsErro = absErr;
        if (absTau > peakTorque) peakTorque = absTau;
        if (absBraco > maxAbsBraco) maxAbsBraco = absBraco;
      }

      const rmsErro = this.window.length > 0 ? Math.sqrt(sumSqErro / this.window.length) : 0;
      const elapsedMs = sample.tMs - this.ctrlStartMs;

      const isStable =
        elapsedMs >= this.thresholds.windowMs &&
        rmsErro < this.thresholds.maxRmsErroDeg &&
        maxAbsErro < this.thresholds.maxPeakErroDeg &&
        maxAbsBraco < this.thresholds.maxBracoDeg;

      const overshootPct = setpointDeg > 0 ? (this.maxOvershootDeg / setpointDeg) * 100.0 : 0;
      const settlingTimeSec =
        this.settledAtMs !== null ? Math.max(0, (this.settledAtMs - this.ctrlStartMs) / 1000.0) : null;

      return {
        state: isStable ? 'ESTAVEL' : 'ESTABILIZANDO',
        rmsErroDeg: rmsErro,
        maxAbsErroDeg: maxAbsErro,
        peakTorqueNm: peakTorque,
        activeDurationSec: this.activeDurationSec,
        bestDurationSec: this.bestDurationSec,
        overshootPct,
        settlingTimeSec,
        peakPoint: this.peakPoint,
        fallReason: null,
      };
    }

    // ctrl == false
    if (this.wasCtrlActive) {
      this.wasCtrlActive = false;
      this.fallen = true;
      if (!this.fallReason) {
        this.fallReason =
          Math.abs(sample.bracoDeg) >= 129.0
            ? 'Braço atingiu limite do cabo (±130°)'
            : 'Haste saiu da zona ativa (180° ± 30°)';
      }
      this.window = [];
      this.ctrlStartMs = null;
    }

    return {
      state: this.fallen ? 'CAIU' : 'DESARMADO',
      rmsErroDeg: 0,
      maxAbsErroDeg: Math.abs(sample.erroDeg),
      peakTorqueNm: 0,
      activeDurationSec: 0,
      bestDurationSec: this.bestDurationSec,
      overshootPct: 0,
      settlingTimeSec: null,
      peakPoint: null,
      fallReason: this.fallReason,
    };
  }
}
