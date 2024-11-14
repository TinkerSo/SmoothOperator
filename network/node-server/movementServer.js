// server.js
const WebSocket = require('ws');
const PORT = 8080;

const server = new WebSocket.Server({ port: PORT });
console.log(`WebSocket server started on ws://localhost:${PORT}`);

server.on('connection', (socket) => {
  console.log('Client connected');

  // Handle incoming messages from clients
  socket.on('message', (message) => {
    console.log(`Received command: ${message}`);

    // Broadcast the command to all connected clients except the sender
    server.clients.forEach((client) => {
      if (client !== socket && client.readyState === WebSocket.OPEN) {
        client.send(message);
      }
    });
  });

  // Handle client disconnection
  socket.on('close', () => {
    console.log('Client disconnected');
  });
});
