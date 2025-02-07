// const express = require('express');
// const app = express();
// const PORT = 3000; // You can change the port if needed

// // Endpoint to test the connection
// app.get('/', (req, res) => {
//   res.send('Hello from the Node server!');
// });

// app.listen(PORT, () => {
//   console.log(`Server running on http://localhost:${PORT}`);
// });
const express = require('express');
const app = express();
const PORT = 3000; // You can change the port if needed

// Middleware to parse JSON request bodies
app.use(express.json());

// Endpoint to test the connection
app.get('/', (req, res) => {
  res.send('Hello from the Node server!');
});

// API endpoint to handle commands
app.post('/api/command', (req, res) => {
  console.log('Headers:', req.headers);
  console.log('Body:', req.body);
  const { command } = req.body;
  if (!command) {
    return res.status(400).send({ error: 'Command is required' });
  }
  console.log(`Received command: ${command}`);
  res.status(200).send({ message: `Command '${command}' received successfully` });
});


app.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
});
