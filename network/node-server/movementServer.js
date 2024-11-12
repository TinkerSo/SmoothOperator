const dgram = require('dgram');
const WebSocket = require('ws');

const udpClient = dgram.createSocket('udp4');

// Update with your ESP32 IP and port
const ESP32_IP = '192.168.1.4';
const ESP32_PORT = 4000;

// Create a WebSocket server on port 3000
const wss = new WebSocket.Server({ port: 3000 });

console.log('WebSocket server running on ws://localhost:3000');

// Handle incoming WebSocket connections
wss.on('connection', (ws) => {
    console.log('Client 1 connected');

    ws.on('message', (message) => {
        console.log(`Received movement command from Client 1: ${message}`);

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
        console.log('Client 1 disconnected');
    });
});

// Handle errors
udpClient.on('error', (err) => {
    console.error(`UDP Client error: ${err.stack}`);
    udpClient.close();
});
