export interface TelemetrySample {
  tMs: number;
  ctrl: boolean;
  angDeg: number;
  erroDeg: number;
  tauPidNm: number;
  tauTotalNm: number;
  stepHz: number;
  bracoDeg: number;
}

export interface ControllerParams {
  kp: number;
  ki: number;
  kd: number;
  k: number;
  z: number;
  setpointDeg: number;
  zonaDeg: number;
  torqueMaxNm: number;
  sinalMotor: number;
  telemHz: number;
  modoKz: boolean;
}

export interface ControllerEvent {
  code: string;
  detail: string;
  raw: string;
}

export type ParsedMessage =
  | { type: 'T'; sample: TelemetrySample }
  | { type: 'P'; params: ControllerParams }
  | { type: 'E'; event: ControllerEvent }
  | { type: 'corrupt'; raw: string; reason: string }
  | { type: 'raw'; text: string };

export const DEFAULT_LGR = {
  k: 0.015601,
  z: 5.9888,
  setpointDeg: 180.0,
  zonaDeg: 30.0,
  torqueMaxNm: 0.14,
  limiteBracoDeg: 130.0,
  telemHz: 100,
} as const;

/**
 * Calcula Kp, Ki, Kd a partir do método de dois zeros reais e iguais:
 *   C(s) = K * (s + z)^2 / s  =>  Kp = 2*K*z,  Ki = K*z^2,  Kd = K
 */
export function gainsFromKz(k: number, z: number): { kp: number; ki: number; kd: number } {
  return {
    kp: 2 * k * z,
    ki: k * z * z,
    kd: k,
  };
}

export function buildKzCommand(k: number, z: number): string {
  if (!Number.isFinite(k) || !Number.isFinite(z) || k <= 0 || k > 5 || z < 0 || z > 200) {
    throw new Error('Faixa permitida: 0 < K ≤ 5 e 0 ≤ z ≤ 200');
  }
  return `k=${k.toFixed(6)},${z.toFixed(4)}\n`;
}

export function buildPidCommand(kp: number, ki: number, kd: number): string {
  if (
    !Number.isFinite(kp) ||
    !Number.isFinite(ki) ||
    !Number.isFinite(kd) ||
    kp < 0 ||
    kp > 50 ||
    ki < 0 ||
    ki > 500 ||
    kd < 0 ||
    kd > 5 ||
    (kp === 0 && ki === 0 && kd === 0)
  ) {
    throw new Error('Faixa permitida: 0 ≤ Kp ≤ 50, 0 ≤ Ki ≤ 500, 0 ≤ Kd ≤ 5 (não todos zero)');
  }
  return `g=${kp.toFixed(6)},${ki.toFixed(6)},${kd.toFixed(6)}\n`;
}

function parseFiniteNumbers(fields: string[]): number[] | null {
  const nums: number[] = new Array(fields.length);
  for (let i = 0; i < fields.length; i++) {
    const raw = fields[i].trim();
    if (raw === '') return null;
    const n = Number(raw);
    if (!Number.isFinite(n)) return null;
    nums[i] = n;
  }
  return nums;
}

let lastAutoTMs = 0;

export function getAutoTimestampMs(): number {
  const now = Math.round(typeof performance !== 'undefined' ? performance.now() : Date.now());
  if (now > lastAutoTMs) {
    lastAutoTMs = now;
  } else {
    lastAutoTMs += 1;
  }
  return lastAutoTMs;
}

export function resetAutoTimestamp(val = 0): void {
  lastAutoTMs = val;
}

// Regex para linha de log serial do esp32_pid_final.ino:
// ctrl=ON  | pulsos=  1200 | ang= 180.05° | braco=  +0.2° | tau=+0.0123 N.m | step= +150.0 Hz
const ESP32_FINAL_LOG_RE =
  /^ctrl=\s*(ON|OFF)\s*\|\s*pulsos=\s*([+-]?\d+)\s*\|\s*ang=\s*([+-]?[\d.]+)(?:°|deg)?\s*\|\s*braco=\s*([+-]?[\d.]+)(?:°|deg)?\s*\|\s*tau=\s*([+-]?[\d.]+)\s*N\.m\s*\|\s*step=\s*([+-]?[\d.]+)\s*Hz/i;

export function parseLine(rawLine: string): ParsedMessage | null {
  const line = rawLine.replace(/\r$/, '').trim();
  if (line.length === 0) return null;

  if (line.startsWith('T,')) {
    const parts = line.slice(2).split(',');
    if (parts.length !== 8) {
      return { type: 'corrupt', raw: line, reason: `Esperados 8 campos em T, recebidos ${parts.length}` };
    }
    const nums = parseFiniteNumbers(parts);
    if (!nums) {
      return { type: 'corrupt', raw: line, reason: 'Valor não numérico em linha T' };
    }
    const [tMs, ctrlNum, angDeg, erroDeg, tauPidNm, tauTotalNm, stepHz, bracoDeg] = nums;
    if (tMs < 0 || (ctrlNum !== 0 && ctrlNum !== 1) || angDeg < -1 || angDeg > 361) {
      return { type: 'corrupt', raw: line, reason: 'Valores fora de escala física em linha T' };
    }
    return {
      type: 'T',
      sample: {
        tMs,
        ctrl: ctrlNum === 1,
        angDeg,
        erroDeg,
        tauPidNm,
        tauTotalNm,
        stepHz,
        bracoDeg,
      },
    };
  }

  // Suporte a telemetria do esp32_pid_final.ino (115200 baud / formato human-readable de bancada)
  const finalMatch = line.match(ESP32_FINAL_LOG_RE);
  if (finalMatch) {
    const ctrl = finalMatch[1].toUpperCase() === 'ON';
    const angDeg = Number(finalMatch[3]);
    const bracoDeg = Number(finalMatch[4]);
    const tauTotalNm = Number(finalMatch[5]);
    const stepHz = Number(finalMatch[6]);
    const erroDeg = Math.round((DEFAULT_LGR.setpointDeg - angDeg) * 100) / 100;
    return {
      type: 'T',
      sample: {
        tMs: getAutoTimestampMs(),
        ctrl,
        angDeg,
        erroDeg,
        tauPidNm: tauTotalNm,
        tauTotalNm,
        stepHz,
        bracoDeg,
      },
    };
  }

  // Suporte a telemetria formato D,... legado de dashboard.html
  if (line.startsWith('D,')) {
    const parts = line.slice(2).split(',');
    if (parts.length >= 7) {
      const nums = parseFiniteNumbers(parts);
      if (nums && nums.length >= 7) {
        const [angDeg, erroDeg, tauPidNm, tauTotalNm, stepHz, bracoDeg, ctrlNum] = nums;
        return {
          type: 'T',
          sample: {
            tMs: getAutoTimestampMs(),
            ctrl: ctrlNum === 1,
            angDeg,
            erroDeg,
            tauPidNm,
            tauTotalNm,
            stepHz,
            bracoDeg,
          },
        };
      }
    }
  }

  if (line.startsWith('P,')) {
    const parts = line.slice(2).split(',');
    if (parts.length !== 10 && parts.length !== 11) {
      return { type: 'corrupt', raw: line, reason: `Esperados 10 ou 11 campos em P, recebidos ${parts.length}` };
    }
    const nums = parseFiniteNumbers(parts);
    if (!nums) {
      return { type: 'corrupt', raw: line, reason: 'Valor não numérico em linha P' };
    }
    const [kp, ki, kd, k, z, setpointDeg, zonaDeg, torqueMaxNm, sinalMotor, telemHz, modoKzRaw] = nums;
    const modoKz = modoKzRaw !== undefined ? modoKzRaw === 1 : true;
    return {
      type: 'P',
      params: {
        kp,
        ki,
        kd,
        k,
        z,
        setpointDeg,
        zonaDeg,
        torqueMaxNm,
        sinalMotor,
        telemHz,
        modoKz,
      },
    };
  }

  if (line.startsWith('E,')) {
    const rest = line.slice(2);
    const commaIdx = rest.indexOf(',');
    if (commaIdx === -1) {
      return {
        type: 'E',
        event: { code: rest.trim(), detail: '', raw: line },
      };
    }
    return {
      type: 'E',
      event: {
        code: rest.slice(0, commaIdx).trim(),
        detail: rest.slice(commaIdx + 1).trim(),
        raw: line,
      },
    };
  }

  // Reconhecimento de eventos do esp32_pid_final.ino
  if (line.includes('Angulo de referencia zerado')) {
    const isBoot = line.includes('BOTAO BOOT') || line.includes('BOOT');
    return {
      type: 'E',
      event: {
        code: 'ZERO',
        detail: isBoot ? 'BOTAO_BOOT' : 'MANUAL',
        raw: line,
      },
    };
  }

  if (line.startsWith('BOOT OK')) {
    return {
      type: 'E',
      event: {
        code: 'BOOT',
        detail: line.replace(/^BOOT OK\s*[-:]?\s*/, '').trim(),
        raw: line,
      },
    };
  }

  return { type: 'raw', text: line };
}
