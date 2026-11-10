const { app, BrowserWindow, ipcMain, session } = require('electron');
const path = require('node:path');
const fs = require('node:fs');
const os = require('node:os');
const { execFile } = require('node:child_process');

// Garante execução fluida do .AppImage em distros Linux recentes (Ubuntu 24.04+, Fedora, Arch)
if (process.platform === 'linux') {
  app.commandLine.appendSwitch('no-sandbox');
}

const ESP32_USB_VIDS = new Set([
  '10c4', // Silicon Labs CP2102 / CP2104 (ESP32 DevKit v1)
  '1a86', // QinHeng CH340 / CH9102
  '0403', // FTDI FT232
  '303a', // Espressif Native USB (ESP32-S2 / S3 / C3 / C6)
  '4348', // WCH alternativo
]);

const UDEV_RULES_CONTENT = [
  '# Permissoes USB Serial para ESP32 (CP210x, CH340/CH9102, FTDI, Espressif)',
  'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", TAG+="uaccess"',
  'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", TAG+="uaccess"',
  'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", TAG+="uaccess"',
  'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", MODE="0666", TAG+="uaccess"',
  'KERNEL=="ttyUSB[0-9]*", MODE="0666", TAG+="uaccess"',
  'KERNEL=="ttyACM[0-9]*", MODE="0666", TAG+="uaccess"',
  '',
].join('\n');

let mainWindow = null;
let pendingSerialCallback = null;

function normalizeVid(vid) {
  if (!vid) return '';
  const num = Number(vid);
  if (Number.isFinite(num) && num > 0) {
    return num.toString(16).padStart(4, '0').toLowerCase();
  }
  return String(vid).replace(/^0x/i, '').padStart(4, '0').toLowerCase();
}

function checkLinuxPortAccess(portName) {
  if (process.platform !== 'linux') return true;
  const devPath = portName.startsWith('/dev/') ? portName : `/dev/${portName}`;
  try {
    fs.accessSync(devPath, fs.constants.R_OK | fs.constants.W_OK);
    return true;
  } catch {
    return false;
  }
}

function installLinuxUsbPermissions(optionalDevPath = '') {
  return new Promise((resolve) => {
    if (process.platform !== 'linux') {
      resolve({ ok: true, message: 'No Windows as portas COM já possuem permissão de usuário.' });
      return;
    }

    const validDev = /^\/dev\/tty(USB|ACM|S)\d+$/.test(optionalDevPath) ? optionalDevPath : '';
    const tmpRulePath = path.join(os.tmpdir(), `99-esp32-pendulo-${process.pid}.rules`);

    try {
      fs.writeFileSync(tmpRulePath, UDEV_RULES_CONTENT, { mode: 0o644 });
    } catch (err) {
      resolve({ ok: false, message: `Falha ao preparar regra udev temporária: ${err.message}` });
      return;
    }

    const script = [
      'cp "$1" /etc/udev/rules.d/99-esp32-pendulo.rules',
      'chmod 644 /etc/udev/rules.d/99-esp32-pendulo.rules',
      'udevadm control --reload-rules || true',
      'udevadm trigger --subsystem-match=tty || true',
      'if [ -n "$2" ] && [ -e "$2" ]; then chmod a+rw "$2" || true; fi',
    ].join(' && ');

    execFile('pkexec', ['sh', '-c', script, 'esp32-udev-setup', tmpRulePath, validDev], (error) => {
      try {
        fs.unlinkSync(tmpRulePath);
      } catch {
        // ignore
      }
      if (error) {
        resolve({
          ok: false,
          message:
            'Autorização cancelada ou pkexec indisponível. Alternativa manual: sudo usermod -aG dialout $USER',
        });
      } else {
        resolve({
          ok: true,
          message: 'Permissões USB do ESP32 configuradas com sucesso (/etc/udev/rules.d/99-esp32-pendulo.rules).',
        });
      }
    });
  });
}

function enrichPortList(portList) {
  return portList
    .filter((p) => {
      if (process.platform !== 'linux') return true;
      // No Linux prioriza portas reais USB/ACM ou portas seriais ativas, evitando dezenas de ttyS virtuais vazias
      if (/^tty(USB|ACM)/i.test(p.portName)) return true;
      return Boolean(p.vendorId || p.productId);
    })
    .map((p) => {
      const vid = normalizeVid(p.vendorId);
      const pid = normalizeVid(p.productId);
      const devPath =
        process.platform === 'linux' && !p.portName.startsWith('/dev/') ? `/dev/${p.portName}` : p.portName;
      const isEsp32 =
        ESP32_USB_VIDS.has(vid) || /^tty(USB|ACM)/i.test(p.portName) || /cp210|ch340|ch910|usb|jtag|esp32/i.test(p.displayName || '');
      return {
        portId: p.portId,
        portName: p.portName,
        devPath,
        displayName: p.displayName || '',
        vendorId: vid,
        productId: pid,
        isEsp32,
        hasPermission: checkLinuxPortAccess(p.portName),
      };
    });
}

function configureSerialPermissions(ses) {
  ses.setPermissionCheckHandler((_webContents, permission) => {
    if (permission === 'serial') return true;
    return false;
  });

  ses.setDevicePermissionHandler((details) => {
    if (details.deviceType === 'serial') return true;
    return false;
  });

  ses.on('select-serial-port', (event, portList, _webContents, callback) => {
    event.preventDefault();

    if (pendingSerialCallback) {
      try {
        pendingSerialCallback('');
      } catch {
        // ignore
      }
    }
    pendingSerialCallback = callback;

    const enriched = enrichPortList(portList);
    // Se não sobrou nenhuma porta USB/COM após filtrar ttyS vazias, envia a lista bruta caso usuário use adaptador especial
    const finalPorts =
      enriched.length > 0
        ? enriched
        : portList.slice(0, 12).map((p) => ({
            portId: p.portId,
            portName: p.portName,
            devPath: process.platform === 'linux' ? `/dev/${p.portName}` : p.portName,
            displayName: p.displayName || '',
            vendorId: normalizeVid(p.vendorId),
            productId: normalizeVid(p.productId),
            isEsp32: false,
            hasPermission: checkLinuxPortAccess(p.portName),
          }));

    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('serial:choose-port', finalPorts);
    } else {
      pendingSerialCallback = null;
      callback('');
    }
  });
}

ipcMain.handle('serial:confirm-port', async (_event, payload) => {
  const cb = pendingSerialCallback;
  pendingSerialCallback = null;
  if (!cb) return { ok: false };

  const portId = payload && typeof payload.portId === 'string' ? payload.portId : '';
  const devPath = payload && typeof payload.devPath === 'string' ? payload.devPath : '';
  const needsPermFix = Boolean(payload && payload.needsPermFix);

  if (!portId) {
    cb('');
    return { ok: false, cancelled: true };
  }

  if (process.platform === 'linux' && needsPermFix && devPath) {
    const permRes = await installLinuxUsbPermissions(devPath);
    if (!permRes.ok) {
      cb('');
      return permRes;
    }
  }

  cb(portId);
  return { ok: true };
});

ipcMain.handle('usb:setup-linux-permissions', async () => {
  return installLinuxUsbPermissions('');
});

ipcMain.handle('app:get-platform-info', () => ({
  platform: process.platform,
  isDesktopApp: true,
}));

function createMainWindow() {
  mainWindow = new BrowserWindow({
    width: 1440,
    height: 938,
    minWidth: 980,
    minHeight: 720,
    backgroundColor: '#0b1220',
    title: 'Pêndulo Invertido · PID 1 kHz',
    icon: path.join(__dirname, '..', 'build', 'icon.png'),
    autoHideMenuBar: true,
    webPreferences: {
      preload: path.join(__dirname, 'preload.cjs'),
      contextIsolation: true,
      nodeIntegration: false,
      backgroundThrottling: false, // Mantém streaming serial em tempo real mesmo com janela em segundo plano
    },
  });

  const devServerUrl = process.env.VITE_DEV_SERVER_URL;
  if (devServerUrl) {
    mainWindow.loadURL(devServerUrl);
  } else {
    mainWindow.loadFile(path.join(__dirname, '..', 'dist', 'index.html'));
  }

  mainWindow.on('closed', () => {
    if (pendingSerialCallback) {
      try {
        pendingSerialCallback('');
      } catch {
        // ignore
      }
      pendingSerialCallback = null;
    }
    mainWindow = null;
  });
}

app.whenReady().then(() => {
  configureSerialPermissions(session.defaultSession);
  createMainWindow();

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createMainWindow();
    }
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
