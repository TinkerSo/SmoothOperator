const WebSocket = require('ws');
const { SerialPort } = require('serialport'); // Use named import

// WebSocket setup
const SERVER_URL = 'ws://192.168.1.3:8080';
const ws = new WebSocket(SERVER_URL);

// Serial port setup for Jetson Nano (adjust the port name if necessary)
const port = new SerialPort({
  path: '/dev/ttyTHS1',
  baudRate: 115200,
});

// Handle serial port errors
port.on('error', (err) => {
  console.error(`Serial port error: ${err.message}`);
});

// When the WebSocket connection opens
ws.on('open', () => {
  console.log('Connected to the server. Listening for movement commands...');
});

// When a message is received from the WebSocket server
ws.on('message', (message) => {
  const command = message.toString().trim().toUpperCase();

  // Validate command
  if (['W', 'A', 'S', 'D', 'X'].includes(command)) {
    //console.log(`Received movement command: ${command}`);
    // Send the command to the ESP32 via UART
    port.write(`${command}\n`, (err) => {
      if (err) {
        return console.error(`Failed to send command to ESP32: ${err.message}`);
      }
      console.log(`Sent command to ESP32: ${command}`);
    });
  }
});

// Handle WebSocket disconnection
ws.on('close', () => {
  console.log('Disconnected from server.');
  port.close(() => {
    console.log('Serial port closed.');
  });
});

// Gracefully close serial port on script exit
process.on('SIGINT', () => {
  console.log('Closing serial port...');
  port.close(() => {
    console.log('Serial port closed. Exiting.');
    process.exit(0);
  });
});
