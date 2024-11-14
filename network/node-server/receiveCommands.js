const WebSocket = require('ws');

const SERVER_URL = 'ws://192.168.1.3:8080';
const ws = new WebSocket(SERVER_URL);

ws.on('open', () => {
  console.log('Connected to the server. Listening for movement commands...');
});

ws.on('message', (message) => {
  console.log(`Received movement command: ${message}`);

  // Handle the movement logic
});

ws.on('close', () => {
  console.log('Disconnected from server.');
});
