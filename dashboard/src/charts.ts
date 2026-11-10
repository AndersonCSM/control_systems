import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import type { SeriesSnapshot } from './buffer';
import { DEFAULT_LGR } from './protocol';

const AXIS_TEXT = '#8b9bb4';
const H_GRID = 'rgba(148, 163, 184, 0.11)';
const SETPOINT_COLOR = '#fbbf24';
const CURVE_COLOR = '#38bdf8';
const BAND_FILL = 'rgba(56, 189, 248, 0.085)';
const CHART_FONT = '13px Inter, -apple-system, BlinkMacSystemFont, sans-serif';

function formatPtBr(val: number, decimals = 1): string {
  return val.toFixed(decimals).replace('.', ',');
}

export class TimeResponseChart {
  private readonly chart: uPlot;
  private readonly container: HTMLElement;
  private readonly resizeObserver: ResizeObserver;

  private setpointDeg: number = DEFAULT_LGR.setpointDeg;
  private zonaDeg: number = DEFAULT_LGR.zonaDeg;
  private settlingBandDeg = 5.0;
  private windowSec = 10;

  constructor(container: HTMLElement) {
    this.container = container;

    const getSize = (): { width: number; height: number } => {
      const rect = container.getBoundingClientRect();
      return {
        width: Math.max(320, Math.floor(rect.width)),
        height: Math.max(220, Math.floor(rect.height)),
      };
    };

    const initialSize = getSize();

    this.chart = new uPlot(
      {
        ...initialSize,
        padding: [18, 28, 8, 8],
        legend: { show: false },
        cursor: {
          drag: { x: false, y: false },
          points: {
            size: 7,
            fill: CURVE_COLOR,
            stroke: '#0b1220',
            width: 2,
          },
        },
        scales: {
          x: {
            time: false,
            range: (_u, _dMin, dMax) => {
              if (!Number.isFinite(dMax) || dMax <= this.windowSec) {
                return [0, this.windowSec];
              }
              return [dMax - this.windowSec, dMax];
            },
          },
          y: {
            range: (_u, dMin, dMax) => {
              const lo = this.setpointDeg - this.zonaDeg;
              const hi = this.setpointDeg + this.zonaDeg;
              if (!Number.isFinite(dMin) || !Number.isFinite(dMax)) {
                return [lo, hi];
              }
              if (dMin >= lo - 5 && dMax <= hi + 5) {
                return [Math.min(lo, Math.floor(dMin - 2)), Math.max(hi, Math.ceil(dMax + 2))];
              }
              return [0, 360];
            },
          },
        },
        axes: [
          {
            stroke: AXIS_TEXT,
            font: CHART_FONT,
            size: 34,
            gap: 10,
            grid: { show: false },
            ticks: { show: false },
            splits: (u) => {
              const min = u.scales.x.min ?? 0;
              const max = u.scales.x.max ?? this.windowSec;
              const span = max - min;
              const step = span / 5;
              return [0, 1, 2, 3, 4, 5].map((i) => min + i * step);
            },
            values: (_u, splits) =>
              splits.map((v) => {
                const rounded = Math.abs(v - Math.round(v)) < 0.05 ? Math.round(v).toString() : formatPtBr(v, 1);
                return `${rounded} s`;
              }),
          },
          {
            stroke: AXIS_TEXT,
            font: CHART_FONT,
            size: 52,
            gap: 10,
            grid: { stroke: H_GRID, width: 1 },
            ticks: { show: false },
            splits: (u) => {
              const min = u.scales.y.min ?? 150;
              const max = u.scales.y.max ?? 210;
              if (min <= 10 && max >= 350) {
                return [0, 90, 180, 270, 360];
              }
              return [
                this.setpointDeg - 30,
                this.setpointDeg - 15,
                this.setpointDeg,
                this.setpointDeg + 15,
                this.setpointDeg + 30,
              ];
            },
            values: (_u, splits) => splits.map((v) => `${Math.round(v)}°`),
          },
        ],
        series: [
          {},
          {
            label: 'Ângulo θ',
            stroke: CURVE_COLOR,
            width: 2.6,
            points: { show: false },
          },
        ],
        hooks: {
          drawAxes: [
            (u) => {
              const ctx = u.ctx;
              const { left, width, top, height } = u.bbox;

              // 1) Faixa sombreada de acomodação (±5° em torno de 180°)
              const yTop = Math.max(
                top,
                Math.min(top + height, u.valToPos(this.setpointDeg + this.settlingBandDeg, 'y', true)),
              );
              const yBot = Math.max(
                top,
                Math.min(top + height, u.valToPos(this.setpointDeg - this.settlingBandDeg, 'y', true)),
              );
              if (yBot > yTop) {
                ctx.save();
                ctx.fillStyle = BAND_FILL;
                ctx.fillRect(left, yTop, width, yBot - yTop);
                ctx.restore();
              }

              // 2) Linha tracejada amarela do Setpoint (180°) de ponta a ponta
              const ySp = u.valToPos(this.setpointDeg, 'y', true);
              if (ySp >= top && ySp <= top + height) {
                ctx.save();
                ctx.strokeStyle = SETPOINT_COLOR;
                ctx.lineWidth = 2 * devicePixelRatio;
                ctx.setLineDash([8 * devicePixelRatio, 6 * devicePixelRatio]);
                ctx.beginPath();
                ctx.moveTo(left, ySp);
                ctx.lineTo(left + width, ySp);
                ctx.stroke();
                ctx.restore();
              }
            },
          ],
          draw: [
            (u) => {
              // Anotação automática do Pico visível na janela quando na região de controle
              const xs = u.data[0];
              const ys = u.data[1];
              if (!xs || !ys || xs.length < 3) return;

              const xMin = u.scales.x.min ?? 0;
              const xMax = u.scales.x.max ?? this.windowSec;
              const lo = this.setpointDeg - this.zonaDeg;
              const hi = this.setpointDeg + this.zonaDeg;

              // Identifica se a curva já cruzou ou se aproximou do setpoint para destacar o pico de sobressinal
              let bestIdx = -1;
              let bestOvershoot = 0;

              // Detecta sentido inicial na janela visível
              let firstValidY: number | null = null;
              for (let i = 0; i < xs.length; i++) {
                const y = ys[i];
                if (xs[i] >= xMin && xs[i] <= xMax && y != null && y >= lo && y <= hi) {
                  firstValidY = y;
                  break;
                }
              }
              if (firstValidY === null) return;
              const entryBelow = firstValidY <= this.setpointDeg;

              for (let i = 1; i < xs.length - 1; i++) {
                const x = xs[i];
                const y = ys[i];
                if (x < xMin || x > xMax || y == null || y < lo || y > hi) continue;

                const over = entryBelow ? y - this.setpointDeg : this.setpointDeg - y;
                if (over > bestOvershoot && over >= 2.0) {
                  bestOvershoot = over;
                  bestIdx = i;
                }
              }

              if (bestIdx === -1) return;

              const px = u.valToPos(xs[bestIdx], 'x', true);
              const py = u.valToPos(ys[bestIdx]!, 'y', true);
              const { left, width, top, height } = u.bbox;
              if (px < left || px > left + width || py < top || py > top + height) return;

              const ctx = u.ctx;
              const dpr = devicePixelRatio;
              ctx.save();

              // Ponto marcador ciano
              ctx.beginPath();
              ctx.arc(px, py, 4.5 * dpr, 0, Math.PI * 2);
              ctx.fillStyle = CURVE_COLOR;
              ctx.fill();
              ctx.lineWidth = 1.5 * dpr;
              ctx.strokeStyle = '#0b1220';
              ctx.stroke();

              // Rótulo "Pico 186,9°"
              const label = `Pico ${formatPtBr(ys[bestIdx]!, 1)}°`;
              ctx.font = `${Math.round(13 * dpr)}px "Inter", -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif`;
              ctx.fillStyle = '#dce6f5';
              const textWidth = ctx.measureText(label).width;
              const labelX = Math.min(left + width - textWidth - 4 * dpr, Math.max(left + 4 * dpr, px + 8 * dpr));
              const labelY = entryBelow
                ? Math.max(top + 14 * dpr, py - 10 * dpr)
                : Math.min(top + height - 6 * dpr, py + 18 * dpr);
              ctx.fillText(label, labelX, labelY);

              ctx.restore();
            },
          ],
        },
      },
      [new Float64Array(0), new Float64Array(0)],
      container,
    );

    this.resizeObserver = new ResizeObserver(() => {
      const s = getSize();
      if (Math.abs(this.chart.width - s.width) > 1 || Math.abs(this.chart.height - s.height) > 1) {
        this.chart.setSize(s);
      }
    });
    this.resizeObserver.observe(this.container);
  }

  setWindowSec(sec: number): void {
    this.windowSec = sec;
    this.chart.redraw(false, true);
  }

  updateConfig(setpointDeg: number, zonaDeg: number): void {
    this.setpointDeg = setpointDeg;
    this.zonaDeg = zonaDeg;
  }

  render(snap: SeriesSnapshot): void {
    this.chart.setData([snap.tSec, snap.angDeg]);
  }

  destroy(): void {
    this.resizeObserver.disconnect();
    this.chart.destroy();
  }
}
