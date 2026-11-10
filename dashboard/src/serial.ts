export interface WebSerialPort extends EventTarget {
  open(options: { baudRate: number; bufferSize?: number }): Promise<void>;
  close(): Promise<void>;
  readable: ReadableStream<Uint8Array<ArrayBuffer>> | null;
  writable: WritableStream<Uint8Array<ArrayBuffer>> | null;
}

interface WebSerialNavigator extends Navigator {
  serial?: {
    requestPort(): Promise<WebSerialPort>;
    addEventListener(type: 'disconnect', listener: (ev: Event) => void): void;
    removeEventListener(type: 'disconnect', listener: (ev: Event) => void): void;
  };
}

export interface SerialCallbacks {
  onLine: (line: string) => void;
  onStatusChange: (connected: boolean, message?: string) => void;
  onError: (err: string) => void;
}

export function isWebSerialSupported(): boolean {
  return typeof navigator !== 'undefined' && 'serial' in navigator;
}

export class SerialConnection {
  private port: WebSerialPort | null = null;
  private reader: ReadableStreamDefaultReader<string> | null = null;
  private readableClosed: Promise<void> | null = null;
  private keepReading = false;
  private readonly callbacks: SerialCallbacks;
  private readonly handleDisconnectBound: (ev: Event) => void;

  constructor(callbacks: SerialCallbacks) {
    this.callbacks = callbacks;
    this.handleDisconnectBound = (ev: Event) => {
      if (this.port && (ev.target === this.port || ev.target === null)) {
        void this.cleanupAfterDisconnect('Dispositivo USB desconectado');
      }
    };
  }

  get isConnected(): boolean {
    return this.port !== null && this.keepReading;
  }

  async connect(baudRate = 115200): Promise<boolean> {
    const nav = navigator as WebSerialNavigator;
    if (!nav.serial) {
      this.callbacks.onError(
        'Web Serial API indisponível neste navegador. Use Google Chrome, Microsoft Edge ou Opera no desktop.',
      );
      return false;
    }

    if (this.isConnected) {
      await this.disconnect();
    }

    try {
      const port = await nav.serial.requestPort();
      await port.open({ baudRate, bufferSize: 16384 });
      this.port = port;
      this.keepReading = true;
      nav.serial.addEventListener('disconnect', this.handleDisconnectBound);

      this.callbacks.onStatusChange(true, `USB conectado (${baudRate} baud)`);
      void this.readLoop();

      // Solicita linha P,... imediatamente (caso a placa já esteja rodando sem resetar)
      setTimeout(() => {
        if (this.isConnected) {
          void this.send('p\n');
        }
      }, 150);

      return true;
    } catch (err) {
      const msg = err instanceof Error ? err.message : String(err);
      if (!msg.toLowerCase().includes('no port selected') && !msg.toLowerCase().includes('cancelled')) {
        this.callbacks.onError(`Falha ao abrir porta serial: ${msg}`);
      }
      this.port = null;
      this.keepReading = false;
      return false;
    }
  }

  async send(data: string): Promise<void> {
    if (!this.port || !this.port.writable) {
      this.callbacks.onError('Porta serial não conectada');
      return;
    }
    const writer = this.port.writable.getWriter();
    try {
      const encoded = new TextEncoder().encode(data);
      await writer.write(encoded);
    } catch (err) {
      const msg = err instanceof Error ? err.message : String(err);
      this.callbacks.onError(`Erro ao enviar comando: ${msg}`);
    } finally {
      writer.releaseLock();
    }
  }

  async disconnect(): Promise<void> {
    this.keepReading = false;
    const nav = navigator as WebSerialNavigator;
    nav.serial?.removeEventListener('disconnect', this.handleDisconnectBound);

    try {
      if (this.reader) {
        await this.reader.cancel();
      }
      if (this.readableClosed) {
        await this.readableClosed.catch(() => undefined);
      }
      if (this.port) {
        await this.port.close();
      }
    } catch {
      // Ignora erros residuais ao fechar porta
    } finally {
      this.reader = null;
      this.readableClosed = null;
      this.port = null;
      this.callbacks.onStatusChange(false, 'Desconectado');
    }
  }

  private async cleanupAfterDisconnect(reason: string): Promise<void> {
    this.keepReading = false;
    const nav = navigator as WebSerialNavigator;
    nav.serial?.removeEventListener('disconnect', this.handleDisconnectBound);
    this.reader = null;
    this.readableClosed = null;
    this.port = null;
    this.callbacks.onStatusChange(false, reason);
  }

  private async readLoop(): Promise<void> {
    if (!this.port || !this.port.readable) return;

    const decoder = new TextDecoderStream();
    this.readableClosed = this.port.readable.pipeTo(decoder.writable);
    this.reader = decoder.readable.getReader();

    let partial = '';
    try {
      while (this.keepReading && this.reader) {
        const { value, done } = await this.reader.read();
        if (done) break;
        if (!value) continue;

        partial += value;
        let nlIdx = partial.indexOf('\n');
        while (nlIdx !== -1) {
          const line = partial.slice(0, nlIdx);
          partial = partial.slice(nlIdx + 1);
          this.callbacks.onLine(line);
          nlIdx = partial.indexOf('\n');
        }
        if (partial.length > 4096) {
          partial = '';
        }
      }
    } catch (err) {
      if (this.keepReading) {
        const msg = err instanceof Error ? err.message : String(err);
        this.callbacks.onError(`Leitura serial interrompida: ${msg}`);
      }
    } finally {
      try {
        this.reader?.releaseLock();
      } catch {
        // ignore
      }
      if (this.keepReading) {
        await this.cleanupAfterDisconnect('Conexão serial encerrada');
      }
    }
  }
}
