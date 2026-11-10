const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('desktopAPI', {
  isDesktop: true,
  platform: process.platform,
  onChooseSerialPort: (handler) => {
    const listener = (_event, ports) => handler(ports);
    ipcRenderer.on('serial:choose-port', listener);
    return () => ipcRenderer.removeListener('serial:choose-port', listener);
  },
  confirmSerialPort: (selection) => ipcRenderer.invoke('serial:confirm-port', selection),
  setupLinuxUsbPermissions: () => ipcRenderer.invoke('usb:setup-linux-permissions'),
  getPlatformInfo: () => ipcRenderer.invoke('app:get-platform-info'),
});
