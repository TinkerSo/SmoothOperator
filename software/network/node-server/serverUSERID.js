const express = require('express');
const cors = require('cors');
const dgram = require('dgram');
const WebSocket = require('ws');

const app = express();
const PORT = 3000;

// ESP32 Configuration
const ESP32_IP = '192.168.1.8';  // ESP32 IP Address
const ESP32_PORT = 3000;          // ESP32 Listening Port
const udpClient = dgram.createSocket('udp4');

// Enable CORS & JSON Parsing Middleware
app.use(cors());
app.use(express.json());

// ✅ Test Route (Check if the server is running)
app.get('/', (req, res) => {
  res.send('Hello from the Unified Node Server!');
});

// ✅ HTTP API to Send Commands to ESP32
app.post('/api/command', (req, res) => {
  const { command } = req.body;

  if (!command) {
    return res.status(400).send({ error: 'Command is required' });
  }

  console.log(`Received command via HTTP: ${command}`);

  // Forward the command to ESP32 via UDP
  udpClient.send(command, ESP32_PORT, ESP32_IP, (err) => {
    if (err) {
      console.error(`Error sending message to ESP32: ${err}`);
      return res.status(500).send({ error: 'Failed to send command to ESP32' });
    }
    console.log(`✅ Forwarded command to ESP32: ${command}`);
    res.status(200).send({ message: `Command '${command}' sent successfully` });
  });
});

// ✅ Start the HTTP Server (Listening on all interfaces)
const server = app.listen(PORT, '0.0.0.0', () => {
  console.log(`HTTP Server running on http://0.0.0.0:${PORT}`);
});

// ✅ WebSocket Server to Handle Real-Time Commands
const wss = new WebSocket.Server({ server });

console.log(`WebSocket server running on ws://0.0.0.0:${PORT}`);

wss.on('connection', (ws, req) => {
  const clientIP = req.socket.remoteAddress;
  console.log(`WebSocket Client Connected: ${clientIP}`);

  ws.on('message', (message) => {
    console.log(`Received command via WebSocket: ${message}`);

    // Forward the message to ESP32 via UDP
    udpClient.send(message, ESP32_PORT, ESP32_IP, (err) => {
      if (err) {
        console.error(`Error sending message to ESP32: ${err}`);
      } else {
        console.log(`✅ Forwarded WebSocket command to ESP32: ${message}`);
      }
    });
  });

  ws.on('close', () => {
    console.log(`WebSocket Client Disconnected: ${clientIP}`);
  });
});
