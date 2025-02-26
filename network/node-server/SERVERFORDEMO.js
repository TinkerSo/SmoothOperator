const express = require('express');
const cors = require('cors');
const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const app = express();
const PORT = 3000;

// UART Configuration
const ARDUINO_UART_PORT = '/dev/ttyTHS1';
const BAUD_RATE = 115200;

const arduinoPort = new SerialPort({
    path: ARDUINO_UART_PORT,
    baudRate: BAUD_RATE,
    autoOpen: true,
    encoding: 'utf8' // Force UTF-8 decoding at the serial port level
});

// Readline Parser for UART Data
const parser = arduinoPort.pipe(new ReadlineParser({ delimiter: '\n' }));

arduinoPort.on('open', () => {
    console.log(`UART connection established with Arduino on ${ARDUINO_UART_PORT}`);
});

arduinoPort.on('error', (err) => {
    console.error(`SerialPort Error: ${err.message}`);
});

// Directly use parsed UTF-8 data
parser.on('data', (data) => {
    const utf8Data = data.trim(); // Clean any extra whitespace/newlines
    console.log(`Received from Arduino: ${utf8Data}`);

    // Broadcast to WebSocket clients
    wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(utf8Data);
        }
    });
});

// Enable CORS & JSON Parsing Middleware
app.use(cors());
app.use(express.json());

// Test Route
app.get('/', (req, res) => {
    res.send('Hello from the Jetson Nano Node.js Server!');
});

function isValidCommand(command) {
    // Ensure the command is a single character and one of the allowed ones
    return typeof command === 'string' && /^[wasd]$/.test(command);
}

// API to Send Commands to Arduino with UTF-8 Encoding
app.post('/api/arduino', (req, res) => {
    let { command } = req.body;
    
    if (!command || !isValidCommand(command.trim())) {
        return res.status(400).send({ error: 'Invalid command. Only "w", "a", "s", and "d" are allowed.' });
    }
    
    command = command.trim();  // Remove any extra whitespace

    console.log(`Sending command to Arduino: ${command}`);

    arduinoPort.write(`${command}\n`, 'utf8', (err) => {
        if (err) {
            console.error(`Error writing to Arduino: ${err}`);
            return res.status(500).send({ error: 'Failed to send command to Arduino' });
        }
        console.log(`Command sent to Arduino: ${command}`);
        res.status(200).send({ message: `Command '${command}' sent successfully` });
    });
});


// Start HTTP Server
const server = app.listen(PORT, '0.0.0.0', () => {
    console.log(`HTTP Server running on http://0.0.0.0:${PORT}`);
});

// WebSocket Server
const wss = new WebSocket.Server({ server });

console.log(`WebSocket server running on ws://0.0.0.0:${PORT}`);

wss.on('connection', (ws, req) => {
    const clientIP = req.socket.remoteAddress;
    console.log(`WebSocket Client Connected: ${clientIP}`);

    ws.on('message', (message) => {
        console.log(`Received WebSocket command: ${message}`);

        // Directly write UTF-8 encoded data
        arduinoPort.write(`${message}\n`, 'utf8', (err) => {
            if (err) {
                console.error(`Error sending to Arduino: ${err}`);
            } else {
                console.log(`Forwarded WebSocket command to Arduino: ${message}`);
            }
        });
    });

    ws.on('close', () => {
        console.log(`WebSocket Client Disconnected: ${clientIP}`);
    });

    ws.on('error', (err) => {
        console.error(`WebSocket Error: ${err.message}`);
    });
});
