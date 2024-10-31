const WebSocket = require('ws');

// Create a WebSocket server on port 3000
const wss = new WebSocket.Server({ port: 3000 });

console.log('WebSocket server running on ws://localhost:3000');

// Handle incoming connections
wss.on('connection', (ws) => {
    console.log('Client connected');

    // Handle incoming messages
    ws.on('message', (message) => {
        console.log(`Received movement command: ${message}`);
        // You can add additional handling logic here
    });

    // Handle connection closure
    ws.on('close', () => {
        console.log('Client disconnected');
    });
});
