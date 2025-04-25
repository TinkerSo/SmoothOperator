const dgram = require('dgram');
const WebSocket = require('ws');

const udpClient = dgram.createSocket('udp4');
const ESP32_IP = '192.168.1.8';
const ESP32_PORT = 3000;

// Create a WebSocket server on port 3000
const wss = new WebSocket.Server({ port: 3000 });

console.log('WebSocket server running on ws://localhost:3000');

// Handle incoming WebSocket connections
wss.on('connection', (ws) => {
    console.log('Client connected');

    ws.on('message', (message) => {
        console.log(`Received movement command: ${message}`);

        // Forward the message to the ESP32 via UDP
        udpClient.send(message, ESP32_PORT, ESP32_IP, (err) => {
            if (err) {
                console.error(`Error sending message to ESP32: ${err}`);
            } else {
                console.log(`Forwarded command to ESP32: ${message}`);
            }
        });
    });

    ws.on('close', () => {
        console.log('Client disconnected');
    });
});
