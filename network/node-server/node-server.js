const express = require('express');
const app = express();
const PORT = 3000; // You can change the port if needed

// Endpoint to test the connection
app.get('/', (req, res) => {
  res.send('Hello from the Node server!');
});

app.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
});
