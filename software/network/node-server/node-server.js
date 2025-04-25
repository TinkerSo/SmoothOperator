const express = require('express');
const cors = require('cors'); // Import CORS middleware

const app = express();
const PORT = 3000; // Change if needed

// Enable CORS for all origins
app.use(cors());
app.use(express.json()); // Middleware to parse JSON

// Test Route
app.get('/', (req, res) => {
  res.send('Hello from the Node server!');
});

// Command Route
app.post('/api/command', (req, res) => {
  console.log('Received request:', req.body);
  const { command } = req.body;
  
  if (!command) {
    return res.status(400).send({ error: 'Command is required' });
  }
  
  console.log(`Received command: ${command}`);
  res.status(200).send({ message: `Command '${command}' received successfully` });
});

// Start Server (Listening on All Network Interfaces)
app.listen(PORT, '0.0.0.0', () => {
  console.log(`Server running on http://0.0.0.0:${PORT}`);
});
