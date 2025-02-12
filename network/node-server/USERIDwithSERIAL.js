const express = require('express');
const cors = require('cors');
const WebSocket = require('ws');
const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');

const app = express();
const PORT = 3000;

// **Serial Configuration for ESP32**
const ESP32_SERIAL_PORT = '/dev/ttyTHS1';  // Change if needed
const BAUD_RATE = 115200;

// **Initialize Serial Connection with ESP32**
const esp32Serial = new SerialPort(ESP32_SERIAL_PORT, { baudRate: BAUD_RATE });
const parser = esp32Serial.pipe(new Readline({ delimiter: '\n' }));

// **Check if Serial Connection is Open**
esp32Serial.on('open', () => {
    console.log(`✅ Serial connection established on ${ESP32_SERIAL_PORT}`);
});

esp32Serial.on('error', (err) => {
    console.error(`❌ Serial port error: ${err.message}`);
});

// **Listen for Responses from ESP32 (Debugging)**
parser.on('data', (data) => {
    console.log(`📩 Data from ESP32: ${data}`);
});

// **Enable CORS & JSON Parsing Middleware**
app.use(cors());
app.use(express.json());

// ✅ **Test Route (Check if Server is Running)**
app.get('/', (req, res) => {
  res.send('Hello from the Jetson Nano Serial Server!');
});

// ✅ **HTTP API to Send Commands to ESP32**
app.post('/api/command', (req, res) => {
  const { command } = req.body;

  if (!command) {
    return res.status(400).send({ error: 'Command is required' });
  }

  console.log(`➡️ Received HTTP command: ${command}`);

  // **Send the command over Serial to ESP32**
  esp32Serial.write(command + '\n', (err) => {
    if (err) {
      console.error(`❌ Error sending command to ESP32: ${err.message}`);
      return res.status(500).send({ error: 'Failed to send command to ESP32' });
    }
    console.log(`✅ Command '${command}' sent to ESP32`);
    res.status(200).send({ message: `Command '${command}' sent successfully` });
  });
});

// ✅ **Start the HTTP Server**
const server = app.listen(PORT, '0.0.0.0', () => {
  console.log(`🌐 HTTP Server running on http://0.0.0.0:${PORT}`);
});

// ✅ **WebSocket Server for Real-Time Commands**
const wss = new WebSocket.Server({ server });

console.log(`📡 WebSocket server running on ws://0.0.0.0:${PORT}`);

wss.on('connection', (ws, req) => {
    const clientIP = req.socket.remoteAddress;
    console.log(`📶 WebSocket Client Connected: ${clientIP}`);

    ws.on('message', (message) => {
        console.log(`➡️ Received WebSocket command: ${message}`);

        // **Send the command over Serial to ESP32**
        esp32Serial.write(message + '\n', (err) => {
            if (err) {
                console.error(`❌ Error sending command to ESP32: ${err.message}`);
            } else {
                console.log(`✅ Command '${message}' sent to ESP32 via Serial`);
            }
        });
    });

    ws.on('close', () => {
        console.log(`❌ WebSocket Client Disconnected: ${clientIP}`);
    });
});
