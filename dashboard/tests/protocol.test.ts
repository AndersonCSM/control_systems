import { describe, expect, it } from 'vitest';
import {
  buildKzCommand,
  buildPidCommand,
  gainsFromKz,
  parseLine,
} from '../src/protocol';

describe('protocol', () => {
  it('calculates Kp, Ki, Kd from K and z (two equal real zeros)', () => {
    const g = gainsFromKz(0.01851, 6.364);
    expect(g.kp).toBeCloseTo(2 * 0.01851 * 6.364, 6);
    expect(g.ki).toBeCloseTo(0.01851 * 6.364 * 6.364, 6);
    expect(g.kd).toBeCloseTo(0.01851, 6);
  });

  it('parses valid T telemetry line', () => {
    const res = parseLine('T,1250,1,180.15,-0.15,0.0124,-0.0118,-285.4,3.2\r\n');
    expect(res).toEqual({
      type: 'T',
      sample: {
        tMs: 1250,
        ctrl: true,
        angDeg: 180.15,
        erroDeg: -0.15,
        tauPidNm: 0.0124,
        tauTotalNm: -0.0118,
        stepHz: -285.4,
        bracoDeg: 3.2,
      },
    });
  });

  it('parses valid P parameters line (11 fields with modoKz)', () => {
    const res = parseLine('P,0.186863,0.559522,0.015601,0.015601,5.9888,180.0,30.0,0.140,-1,100,1');
    expect(res).toEqual({
      type: 'P',
      params: {
        kp: 0.186863,
        ki: 0.559522,
        kd: 0.015601,
        k: 0.015601,
        z: 5.9888,
        setpointDeg: 180.0,
        zonaDeg: 30.0,
        torqueMaxNm: 0.14,
        sinalMotor: -1,
        telemHz: 100,
        modoKz: true,
      },
    });
  });

  it('parses E event lines with and without detail', () => {
    expect(parseLine('E,CTRL_ON')).toEqual({
      type: 'E',
      event: { code: 'CTRL_ON', detail: '', raw: 'E,CTRL_ON' },
    });

    expect(parseLine('E,CTRL_OFF,Haste saiu da zona do topo (±30 deg)')).toEqual({
      type: 'E',
      event: {
        code: 'CTRL_OFF',
        detail: 'Haste saiu da zona do topo (±30 deg)',
        raw: 'E,CTRL_OFF,Haste saiu da zona do topo (±30 deg)',
      },
    });
  });

  it('flags truncated or NaN T/P lines as corrupt without throwing', () => {
    const truncated = parseLine('T,1250,1,180.15');
    expect(truncated?.type).toBe('corrupt');

    const badNum = parseLine('T,1250,1,abc,0,0,0,0,0');
    expect(badNum?.type).toBe('corrupt');

    const badCtrl = parseLine('T,1250,9,180.0,0,0,0,0,0');
    expect(badCtrl?.type).toBe('corrupt');
  });

  it('preserves LOG.TXT lines as raw console lines', () => {
    const header = parseLine('================ INICIO LOG.TXT (COPIE OU SALVE) ================');
    expect(header).toEqual({
      type: 'raw',
      text: '================ INICIO LOG.TXT (COPIE OU SALVE) ================',
    });
    const csvRow = parseLine('25,179.85,0.15,0.0042,-0.0040,-95.2,0.4');
    expect(csvRow?.type).toBe('raw');
  });

  it('builds validated k= and g= commands and rejects invalid ranges', () => {
    expect(buildKzCommand(0.01851, 6.364)).toBe('k=0.018510,6.3640\n');
    expect(() => buildKzCommand(0, 5)).toThrow();
    expect(() => buildKzCommand(-1, 5)).toThrow();
    expect(() => buildKzCommand(10, 5)).toThrow();

    expect(buildPidCommand(0.235595, 0.749664, 0.01851)).toBe('g=0.235595,0.749664,0.018510\n');
    expect(() => buildPidCommand(0, 0, 0)).toThrow();
    expect(() => buildPidCommand(-0.1, 1, 0.01)).toThrow();
  });

  it('parses esp32_pid_final.ino human-readable serial log lines (firmware melhorado)', () => {
    const lineOn =
      'ctrl=ON  | pulsos=  1200 | ang= 180.05° | braco=  +0.2° | tau=+0.0123 N.m | step= +150.0 Hz';
    const resOn = parseLine(lineOn);
    expect(resOn?.type).toBe('T');
    if (resOn?.type === 'T') {
      expect(resOn.sample.ctrl).toBe(true);
      expect(resOn.sample.angDeg).toBeCloseTo(180.05, 2);
      expect(resOn.sample.erroDeg).toBeCloseTo(-0.05, 2);
      expect(resOn.sample.bracoDeg).toBeCloseTo(0.2, 1);
      expect(resOn.sample.tauTotalNm).toBeCloseTo(0.0123, 4);
      expect(resOn.sample.stepHz).toBeCloseTo(150.0, 1);
      expect(resOn.sample.firmwareMode).toBe('melhorado');
      expect(resOn.sample.tMs).toBeGreaterThan(0);
    }

    const lineOff =
      'ctrl=OFF | pulsos=     0 | ang=   0.00° | braco=  +0.0° | tau=+0.0000 N.m | step=   +0.0 Hz';
    const resOff = parseLine(lineOff);
    expect(resOff?.type).toBe('T');
    if (resOff?.type === 'T') {
      expect(resOff.sample.ctrl).toBe(false);
      expect(resOff.sample.angDeg).toBeCloseTo(0.0, 2);
      expect(resOff.sample.erroDeg).toBeCloseTo(180.0, 2);
      expect(resOff.sample.tauTotalNm).toBeCloseTo(0.0, 4);
      expect(resOff.sample.stepHz).toBeCloseTo(0.0, 1);
      expect(resOff.sample.firmwareMode).toBe('melhorado');
    }
  });

  it('parses simples.ino human-readable serial log lines (firmware simples sem campo braco)', () => {
    const lineOn =
      'ctrl=ON  | pulsos=  1200 | ang= 180.05° | tau=+0.0123 N.m | step= +150.0 Hz';
    const resOn = parseLine(lineOn);
    expect(resOn?.type).toBe('T');
    if (resOn?.type === 'T') {
      expect(resOn.sample.ctrl).toBe(true);
      expect(resOn.sample.angDeg).toBeCloseTo(180.05, 2);
      expect(resOn.sample.erroDeg).toBeCloseTo(-0.05, 2);
      expect(resOn.sample.bracoDeg).toBe(0);
      expect(resOn.sample.tauTotalNm).toBeCloseTo(0.0123, 4);
      expect(resOn.sample.stepHz).toBeCloseTo(150.0, 1);
      expect(resOn.sample.firmwareMode).toBe('simples');
      expect(resOn.sample.tMs).toBeGreaterThan(0);
    }

    const lineOff =
      'ctrl=OFF | pulsos=     0 | ang=   0.00° | tau=+0.0000 N.m | step=   +0.0 Hz';
    const resOff = parseLine(lineOff);
    expect(resOff?.type).toBe('T');
    if (resOff?.type === 'T') {
      expect(resOff.sample.ctrl).toBe(false);
      expect(resOff.sample.angDeg).toBeCloseTo(0.0, 2);
      expect(resOff.sample.erroDeg).toBeCloseTo(180.0, 2);
      expect(resOff.sample.bracoDeg).toBe(0);
      expect(resOff.sample.tauTotalNm).toBeCloseTo(0.0, 4);
      expect(resOff.sample.stepHz).toBeCloseTo(0.0, 1);
      expect(resOff.sample.firmwareMode).toBe('simples');
    }
  });

  it('parses simples and melhorado boot banners', () => {
    const simplesBoot = 'BOOT OK - Controle 2 SIMPLES (PID Puro + FT Pura)';
    expect(parseLine(simplesBoot)).toEqual({
      type: 'E',
      event: {
        code: 'BOOT',
        detail: 'Controle 2 SIMPLES (PID Puro + FT Pura)',
        raw: simplesBoot,
      },
    });

    const melhoradoBoot = 'BOOT OK - Controle 2 FINAL SLIM (Deriv=15Hz | FreqMax=4000Hz)';
    expect(parseLine(melhoradoBoot)).toEqual({
      type: 'E',
      event: {
        code: 'BOOT',
        detail: 'Controle 2 FINAL SLIM (Deriv=15Hz | FreqMax=4000Hz)',
        raw: melhoradoBoot,
      },
    });
  });

  it('parses esp32_pid_final.ino events: boot and zeroing', () => {
    const bootLine = 'BOOT OK - Controle 2 FINAL (Deriv=15Hz | FreqMax=4000Hz)';
    expect(parseLine(bootLine)).toEqual({
      type: 'E',
      event: {
        code: 'BOOT',
        detail: 'Controle 2 FINAL (Deriv=15Hz | FreqMax=4000Hz)',
        raw: bootLine,
      },
    });

    const bootBtnZero =
      '>>> [BOTAO BOOT] Angulo de referencia zerado na posicao atual (0.00 graus)!';
    expect(parseLine(bootBtnZero)).toEqual({
      type: 'E',
      event: {
        code: 'ZERO',
        detail: 'BOTAO_BOOT',
        raw: bootBtnZero,
      },
    });

    const manualZero =
      '>>> [MANUAL] Angulo de referencia zerado na posicao atual (0.00 graus)!';
    expect(parseLine(manualZero)).toEqual({
      type: 'E',
      event: {
        code: 'ZERO',
        detail: 'MANUAL',
        raw: manualZero,
      },
    });
  });

  it('parses legacy D,... telemetry line', () => {
    const legacyLine = 'D,180.12,-0.12,0.0120,0.0115,120.0,0.5,1,1200';
    const res = parseLine(legacyLine);
    expect(res?.type).toBe('T');
    if (res?.type === 'T') {
      expect(res.sample.ctrl).toBe(true);
      expect(res.sample.angDeg).toBe(180.12);
      expect(res.sample.erroDeg).toBe(-0.12);
      expect(res.sample.stepHz).toBe(120.0);
    }
  });

  it('parses parameters line emitted by esp32_pid_final enviar_parametros', () => {
    const pLine = 'P,0.186863,0.559522,0.015601,0.015601,5.9888,180.0,30.0,0.140,-1,10,1\r\n';
    const res = parseLine(pLine);
    expect(res?.type).toBe('P');
    if (res?.type === 'P') {
      expect(res.params.k).toBeCloseTo(0.015601, 6);
      expect(res.params.z).toBeCloseTo(5.9888, 4);
      expect(res.params.kp).toBeCloseTo(0.186863, 6);
      expect(res.params.ki).toBeCloseTo(0.559522, 6);
      expect(res.params.kd).toBeCloseTo(0.015601, 6);
      expect(res.params.sinalMotor).toBe(-1);
      expect(res.params.modoKz).toBe(true);
    }
  });

  it('parses autocal, invert, and gain update events', () => {
    expect(parseLine('E,AUTOCAL_DONE,-1')).toEqual({
      type: 'E',
      event: { code: 'AUTOCAL_DONE', detail: '-1', raw: 'E,AUTOCAL_DONE,-1' },
    });
    expect(parseLine('E,INVERT,1')).toEqual({
      type: 'E',
      event: { code: 'INVERT', detail: '1', raw: 'E,INVERT,1' },
    });
    expect(parseLine('E,GAINS_KZ,0.015601,5.9888')).toEqual({
      type: 'E',
      event: { code: 'GAINS_KZ', detail: '0.015601,5.9888', raw: 'E,GAINS_KZ,0.015601,5.9888' },
    });
  });
});
