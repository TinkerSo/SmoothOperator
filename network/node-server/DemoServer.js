const express = require('express');
const cors = require('cors');
const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const app = express();
const PORT = 3000;

// UART Configuration
const ARDUINO_UART_PORT = '/dev/ttyACM0';
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
app.use(express.text()); 

// Test Route
app.get('/', (req, res) => {
    res.send('Hello from the Jetson Nano Node.js Server!');
});

// Function to map commands to vCommand strings
// Flipped the values to match the physical robot.
function getVCommand(command) {
    switch (command) {
        case 'w': return "-0.015 0.000 0.000";
        case 'a': return "0.000 0.000 0.015";
        case 'd': return "0.000 0.000 -0.015";
        case 's': return "0.015 0.000 0.000";
        case 'x': return "0.000 0.000 0.000";
        default: return null;
    }
}

// API to Send Commands to Arduino with UTF-8 Encoding
app.post('/api/arduino', (req, res) => {
    let { command } = req.body;
    
    if (!command || !getVCommand(command.trim())) {
        return res.status(400).send({ error: 'Invalid command. Only "w", "a", "s", "d", and "x" are allowed.' });
    }
    
    command = command.trim();  // Remove any extra whitespace
    const vCommand = getVCommand(command);

    console.log(`Sending command to Arduino: ${vCommand}`);

    arduinoPort.write(`${vCommand}\n`, 'utf8', (err) => {
        if (err) {
            console.error(`Error writing to Arduino: ${err}`);
            return res.status(500).send({ error: 'Failed to send command to Arduino' });
        }
        console.log(`Command sent to Arduino: ${vCommand}`);
        res.status(200).send({ message: `Command '${command}' sent successfully` });
    });
});

// API endpoint for ROS robot_command listener to send vx, vy, vtheta for autonomous drive
app.post('/api/ros', (req, res) => {
    const data = req.body.trim();  // Extract raw text data

    if (!data) {
        return res.status(400).send({ error: 'Invalid data format' });
    }

    console.log(`Received from ROS: ${data}`);
    arduinoPort.write(`${data}\n`, 'utf8', (err) => {
        if (err) {
            console.error(`Error writing to Arduino: ${err}`);
            return res.status(500).send({ error: 'Failed to send ROS command to Arduino' });
        }
        console.log(`ROS command sent to Arduino: ${data}`);
        res.status(200).send({ message: `ROS command sent successfully` });
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
        const vCommand = getVCommand(message.toString().trim());

        if (!vCommand) {
            console.error(`Invalid WebSocket command: ${message}`);
            return;
        }

        // Directly write UTF-8 encoded data
        arduinoPort.write(`${vCommand}\n`, 'utf8', (err) => {
            if (err) {
                console.error(`Error sending to Arduino: ${err}`);
            } else {
                console.log(`Forwarded WebSocket command to Arduino: ${vCommand}`);
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