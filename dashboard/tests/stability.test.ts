import { describe, expect, it } from 'vitest';
import { TelemetryBuffer } from '../src/buffer';
import { StabilityDetector } from '../src/stability';
import { PendulumSimulator } from '../src/simulator';
import { parseLine } from '../src/protocol';

describe('TelemetryBuffer', () => {
  it('wraps around capacity and maintains monotonic time even across MCU reset', () => {
    const buf = new TelemetryBuffer(5);
    for (let i = 0; i < 7; i++) {
      buf.push({
        tMs: i * 10,
        ctrl: true,
        angDeg: 180 + i * 0.1,
        erroDeg: -i * 0.1,
        tauPidNm: 0,
        tauTotalNm: 0,
        stepHz: 0,
        bracoDeg: 0,
      });
    }
    expect(buf.size).toBe(5);
    const snap = buf.snapshot();
    expect(Array.from(snap.tSec)).toEqual([0.02, 0.03, 0.04, 0.05, 0.06]);

    // Simula reboot do ESP32 (tMs volta para 0)
    buf.push({
      tMs: 0,
      ctrl: false,
      angDeg: 0,
      erroDeg: 180,
      tauPidNm: 0,
      tauTotalNm: 0,
      stepHz: 0,
      bracoDeg: 0,
    });
    const snap2 = buf.snapshot();
    expect(snap2.tSec[4]).toBeGreaterThan(snap2.tSec[3]);
    expect(buf.toCsv().split('\n').length).toBeGreaterThan(8);
  });
});

describe('StabilityDetector', () => {
  it('transitions DESARMADO -> ESTABILIZANDO -> ESTAVEL -> CAIU', () => {
    const det = new StabilityDetector();

    // Haste embaixo
    let rep = det.update({
      tMs: 0,
      ctrl: false,
      angDeg: 0,
      erroDeg: 180,
      tauPidNm: 0,
      tauTotalNm: 0,
      stepHz: 0,
      bracoDeg: 0,
    });
    expect(rep.state).toBe('DESARMADO');

    // Acabou de entrar no topo (165°) com erro inicial de 15 graus
    rep = det.update({
      tMs: 100,
      ctrl: true,
      angDeg: 165,
      erroDeg: 15,
      tauPidNm: 0.05,
      tauTotalNm: -0.05,
      stepHz: -400,
      bracoDeg: 2,
    });
    expect(rep.state).toBe('ESTABILIZANDO');
    expect(rep.settlingTimeSec).toBeNull();

    // Pico de sobressinal em 186.9° aos 600 ms
    rep = det.update({
      tMs: 600,
      ctrl: true,
      angDeg: 186.9,
      erroDeg: -6.9,
      tauPidNm: -0.04,
      tauTotalNm: 0.04,
      stepHz: 320,
      bracoDeg: 5,
    });
    expect(rep.overshootPct).toBeCloseTo((6.9 / 180) * 100, 2);
    expect(rep.peakPoint?.angDeg).toBeCloseTo(186.9, 2);

    // Acomoda dentro de ±5° em t = 1300 ms (1.2 s após t=100 ms) e permanece até 3400 ms
    for (let t = 1300; t <= 3400; t += 10) {
      rep = det.update({
        tMs: t,
        ctrl: true,
        angDeg: 179.7,
        erroDeg: 0.3,
        tauPidNm: 0.004,
        tauTotalNm: -0.004,
        stepHz: -20,
        bracoDeg: 4,
      });
    }
    expect(rep.state).toBe('ESTAVEL');
    expect(rep.rmsErroDeg).toBeLessThan(2.0);
    expect(rep.settlingTimeSec).toBeCloseTo(1.2, 2);
    expect(rep.activeDurationSec).toBeCloseTo(3.3, 1);

    // Derrubou a haste
    det.notifyFallReason('Haste saiu da zona do topo (±30 deg)');
    rep = det.update({
      tMs: 3410,
      ctrl: false,
      angDeg: 140,
      erroDeg: 40,
      tauPidNm: 0,
      tauTotalNm: 0,
      stepHz: 0,
      bracoDeg: 10,
    });
    expect(rep.state).toBe('CAIU');
    expect(rep.fallReason).toContain('zona do topo');
  });
});

describe('PendulumSimulator closed-loop physics', () => {
  it('stabilizes around 180 deg with the LGR controller K=0.01851, z=6.364', () => {
    const lines: string[] = [];
    const sim = new PendulumSimulator((line) => lines.push(line));
    const det = new StabilityDetector();

    sim.advanceMs(3500); // 3.5 segundos simulados a 1 kHz

    let lastReport = null;
    let lastSample = null;
    for (const line of lines) {
      const parsed = parseLine(line);
      if (parsed?.type === 'T') {
        lastSample = parsed.sample;
        lastReport = det.update(parsed.sample);
      }
    }

    expect(lastSample).not.toBeNull();
    expect(lastSample!.ctrl).toBe(true);
    expect(Math.abs(180 - lastSample!.angDeg)).toBeLessThan(1.5);
    expect(lastReport?.state).toBe('ESTAVEL');
  });

  it('stabilizes around 180 deg in simples firmware mode without arm feedback', () => {
    const lines: string[] = [];
    const sim = new PendulumSimulator((line) => lines.push(line));
    sim.setFirmwareMode('simples');
    const det = new StabilityDetector();

    sim.advanceMs(3500); // 3.5 segundos simulados a 1 kHz

    let lastReport = null;
    let lastSample = null;
    for (const line of lines) {
      const parsed = parseLine(line);
      if (parsed?.type === 'T') {
        lastSample = parsed.sample;
        lastReport = det.update(parsed.sample);
      }
    }

    expect(lastSample).not.toBeNull();
    expect(lastSample!.ctrl).toBe(true);
    expect(Math.abs(180 - lastSample!.angDeg)).toBeLessThan(2.0);
    expect(lastReport?.state).toBe('ESTAVEL');
  });
});
