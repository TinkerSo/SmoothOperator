const dgram = require('dgram');
const WebSocket = require('ws');

const udpClient = dgram.createSocket('udp4');
const ESP32_IP = '192.168.1.8'; // Keep ESP32 IP
const ESP32_PORT = 3000;

// WebSocket server listening on ALL network interfaces
const wss = new WebSocket.Server({ host: '0.0.0.0', port: 3000 });

console.log('WebSocket server running on ws://0.0.0.0:3000');

// Handle WebSocket connections
wss.on('connection', (ws, req) => {
    const clientIP = req.socket.remoteAddress;
    console.log(`Client connected from ${clientIP}`);

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
        console.log(`Client ${clientIP} disconnected`);
    });
});
