import type { TelemetrySample } from './protocol';

export interface SeriesSnapshot {
  tSec: Float64Array;
  angDeg: Float64Array;
  setpointDeg: Float64Array;
  erroDeg: Float64Array;
  tauPidNm: Float64Array;
  tauTotalNm: Float64Array;
  stepHz: Float64Array;
  bracoDeg: Float64Array;
}

/**
 * Buffer circular sem alocação por amostra para plotagem fluida a 60 fps,
 * mais histórico linear limitado para exportação CSV da sessão.
 */
export class TelemetryBuffer {
  private readonly capacity: number;
  private head = 0;
  private count = 0;
  private t0Ms: number | null = null;
  private lastTMs: number | null = null;
  private timeOffsetMs = 0;

  private readonly tSecBuf: Float64Array;
  private readonly angBuf: Float64Array;
  private readonly spBuf: Float64Array;
  private readonly erroBuf: Float64Array;
  private readonly tauPidBuf: Float64Array;
  private readonly tauTotBuf: Float64Array;
  private readonly stepBuf: Float64Array;
  private readonly bracoBuf: Float64Array;

  // Buffers de saída pré-alocados para snapshot (evita GC a 60 fps)
  private readonly outTSec: Float64Array;
  private readonly outAng: Float64Array;
  private readonly outSp: Float64Array;
  private readonly outErro: Float64Array;
  private readonly outTauPid: Float64Array;
  private readonly outTauTot: Float64Array;
  private readonly outStep: Float64Array;
  private readonly outBraco: Float64Array;

  private readonly csvLog: TelemetrySample[] = [];
  private readonly maxCsvSamples: number;

  constructor(capacity = 2000, maxCsvSamples = 60_000) {
    this.capacity = capacity;
    this.maxCsvSamples = maxCsvSamples;

    this.tSecBuf = new Float64Array(capacity);
    this.angBuf = new Float64Array(capacity);
    this.spBuf = new Float64Array(capacity);
    this.erroBuf = new Float64Array(capacity);
    this.tauPidBuf = new Float64Array(capacity);
    this.tauTotBuf = new Float64Array(capacity);
    this.stepBuf = new Float64Array(capacity);
    this.bracoBuf = new Float64Array(capacity);

    this.outTSec = new Float64Array(capacity);
    this.outAng = new Float64Array(capacity);
    this.outSp = new Float64Array(capacity);
    this.outErro = new Float64Array(capacity);
    this.outTauPid = new Float64Array(capacity);
    this.outTauTot = new Float64Array(capacity);
    this.outStep = new Float64Array(capacity);
    this.outBraco = new Float64Array(capacity);
  }

  push(sample: TelemetrySample, setpointDeg = 180.0): number {
    if (this.t0Ms === null) {
      this.t0Ms = sample.tMs;
      this.lastTMs = sample.tMs;
      this.timeOffsetMs = 0;
    } else if (this.lastTMs !== null && sample.tMs < this.lastTMs) {
      // Caso o ESP32 reinicie durante a sessão, mantém o eixo X monotônico
      this.timeOffsetMs += this.lastTMs - this.t0Ms + 10;
      this.t0Ms = sample.tMs;
    }
    this.lastTMs = sample.tMs;

    const relSec = (this.timeOffsetMs + (sample.tMs - this.t0Ms)) / 1000.0;
    const idx = this.head;

    this.tSecBuf[idx] = relSec;
    this.angBuf[idx] = sample.angDeg;
    this.spBuf[idx] = setpointDeg;
    this.erroBuf[idx] = sample.erroDeg;
    this.tauPidBuf[idx] = sample.tauPidNm;
    this.tauTotBuf[idx] = sample.tauTotalNm;
    this.stepBuf[idx] = sample.stepHz;
    this.bracoBuf[idx] = sample.bracoDeg;

    this.head = (idx + 1) % this.capacity;
    if (this.count < this.capacity) {
      this.count++;
    }

    if (this.csvLog.length < this.maxCsvSamples) {
      this.csvLog.push(sample);
    } else {
      this.csvLog.shift();
      this.csvLog.push(sample);
    }

    return relSec;
  }

  clear(): void {
    this.head = 0;
    this.count = 0;
    this.t0Ms = null;
    this.lastTMs = null;
    this.timeOffsetMs = 0;
    this.csvLog.length = 0;
  }

  get size(): number {
    return this.count;
  }

  get totalRecorded(): number {
    return this.csvLog.length;
  }

  /**
   * Retorna fatia ordenada cronologicamente dos últimos `windowSec` segundos.
   */
  snapshot(windowSec?: number): SeriesSnapshot {
    const n = this.count;
    if (n === 0) {
      const empty = new Float64Array(0);
      return {
        tSec: empty,
        angDeg: empty,
        setpointDeg: empty,
        erroDeg: empty,
        tauPidNm: empty,
        tauTotalNm: empty,
        stepHz: empty,
        bracoDeg: empty,
      };
    }

    const start = this.count === this.capacity ? this.head : 0;
    for (let i = 0; i < n; i++) {
      const src = (start + i) % this.capacity;
      this.outTSec[i] = this.tSecBuf[src];
      this.outAng[i] = this.angBuf[src];
      this.outSp[i] = this.spBuf[src];
      this.outErro[i] = this.erroBuf[src];
      this.outTauPid[i] = this.tauPidBuf[src];
      this.outTauTot[i] = this.tauTotBuf[src];
      this.outStep[i] = this.stepBuf[src];
      this.outBraco[i] = this.bracoBuf[src];
    }

    let firstIdx = 0;
    if (windowSec !== undefined && windowSec > 0) {
      const minT = this.outTSec[n - 1] - windowSec;
      while (firstIdx < n - 1 && this.outTSec[firstIdx] < minT) {
        firstIdx++;
      }
    }

    return {
      tSec: this.outTSec.subarray(firstIdx, n),
      angDeg: this.outAng.subarray(firstIdx, n),
      setpointDeg: this.outSp.subarray(firstIdx, n),
      erroDeg: this.outErro.subarray(firstIdx, n),
      tauPidNm: this.outTauPid.subarray(firstIdx, n),
      tauTotalNm: this.outTauTot.subarray(firstIdx, n),
      stepHz: this.outStep.subarray(firstIdx, n),
      bracoDeg: this.outBraco.subarray(firstIdx, n),
    };
  }

  toCsv(): string {
    const lines = [
      't_ms,ctrl,ang_deg,erro_deg,tau_pid_nm,tau_total_nm,step_hz,braco_deg',
    ];
    for (const s of this.csvLog) {
      lines.push(
        `${s.tMs},${s.ctrl ? 1 : 0},${s.angDeg.toFixed(2)},${s.erroDeg.toFixed(2)},${s.tauPidNm.toFixed(4)},${s.tauTotalNm.toFixed(4)},${s.stepHz.toFixed(1)},${s.bracoDeg.toFixed(1)}`,
      );
    }
    return lines.join('\n') + '\n';
  }
}
