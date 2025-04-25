const express = require('express');
const cors = require('cors');
const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const app = express();
const PORT = 3000;

// UART Configuration
const ARDUINO_UART_PORT = '/dev/ttyACM0';
const BAUD_RATE = 9600;

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

// Directly use parsed UTF-8 data from Arduino and broadcast to WebSocket clients
parser.on('data', (data) => {
    const utf8Data = data.trim();
    console.log(`Received from Arduino: ${utf8Data}`);
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

// Function to map commands to vCommand strings (flipped for the physical robot)
function getVCommand(command) {
    switch (command) {
        case 'w': return "-0.150 0.000 0.000 0.000";
        case 'a': return "0.000 0.000 -0.100 0.000";
        case 'd': return "0.000 0.000 0.100 0.000";
        case 's': return "0.150 0.000 0.000 0.000";
        case 'x': return "0.000 0.000 0.000 0.000";
        case '+': return "0.000 0.000 0.000 1.000";
        case '-': return "0.000 0.000 0.000 -1.000";
        case '=': return "0.000 0.000 0.000 0.000";
        default: return null;
    }
}

// API endpoint for ROS commands
// app.post('/api/ros', (req, res) => {
//     const data = req.body.trim();
//     if (!data) {
//         return res.status(400).send({ error: 'Invalid data format' });
//     }
//     console.log(`Received from ROS: ${data}`);
//     arduinoPort.write(`${data}\n`, 'utf8', (err) => {
//         if (err) {
//             console.error(`Error writing to Arduino: ${err}`);
//             return res.status(500).send({ error: 'Failed to send ROS command to Arduino' });
//         }
//         // Use drain() to ensure the serial command is fully sent
//         arduinoPort.drain(() => {
//             console.log(`Drain complete for ROS command: ${data}`);
//             res.status(200).send({ message: `ROS command sent successfully` });
//         });
//     });
// });


// API endpoint for ROS commands WITH CAP VTHETA TO 0.150
app.post('/api/ros', (req, res) => {
    const data = req.body.trim();
    if (!data) {
        return res.status(400).send({ error: 'Invalid data format' });
    }

    const parts = data.split(' ');
    if (parts.length !== 4) {
        return res.status(400).send({ error: 'Expected 4 float values separated by spaces' });
    }

    let [Vx, Vy, Vtheta, lift] = parts.map(parseFloat);

    // Cap Vtheta if it's greater than 0.150
    if (Vtheta > 0.150) {
        console.log(`Capping Vtheta from ${Vtheta} to 0.150`);
        Vtheta = 0.150;
    }

    const cappedCommand = `${Vx.toFixed(3)} ${Vy.toFixed(3)} ${Vtheta.toFixed(3)} ${lift.toFixed(3)}`;
    console.log(`Received from ROS: ${data}`);
    console.log(`Sending to Arduino: ${cappedCommand}`);

    arduinoPort.write(`${cappedCommand}\n`, 'utf8', (err) => {
        if (err) {
            console.error(`Error writing to Arduino: ${err}`);
            return res.status(500).send({ error: 'Failed to send ROS command to Arduino' });
        }

        // Use drain() to ensure the serial command is fully sent
        arduinoPort.drain(() => {
            console.log(`Drain complete for ROS command: ${cappedCommand}`);
            res.status(200).send({ message: 'ROS command sent successfully' });
        });
    });
});


// Global variable to store the current passcode
let currentPasscode = null;

// Endpoint to receive the passcode from the robot
app.post('/api/connect', (req, res) => {
    const { passcode } = req.body;
    if (!passcode) {
        return res.status(400).send({ error: 'Passcode is required.' });
    }
    currentPasscode = passcode.trim();
    console.log(`Robot set passcode: ${currentPasscode}`);
    return res.status(200).send({ message: 'Passcode received.', passcode: currentPasscode });
});

// Endpoint to authenticate the React Native app's connection attempt
app.post('/api/authenticate', (req, res) => {
    const { passcode } = req.body;
    if (!currentPasscode) {
        return res.status(400).send({ error: 'Robot has not set a passcode yet.' });
    }
    if (!passcode) {
        return res.status(400).send({ error: 'Passcode is required.' });
    }
    if (passcode.trim() === currentPasscode) {
        console.log(`React Native authenticated successfully with passcode: ${passcode}`);
        return res.status(200).send({ message: 'Authenticated' });
    } else {
        console.log(`React Native failed authentication: ${passcode} does not match ${currentPasscode}`);
        return res.status(401).send({ error: 'Incorrect passcode' });
    }
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
        const trimmedMessage = message.toString().trim();
        
        // If the message is "AUTH_SUCCESS", broadcast it.
        if (trimmedMessage === 'AUTH_SUCCESS') {
            console.log("Received AUTH_SUCCESS. Broadcasting to all clients...");
            wss.clients.forEach((client) => {
                if (client.readyState === WebSocket.OPEN) {
                    client.send('AUTH_SUCCESS');
                }
            });
            return; // Do not process further.
        }

        // Check if it's a movement command
        if (['w', 'a', 's', 'd', 'x', '+', '=', '-'].includes(trimmedMessage)) {
            console.log(`Received movement command: ${trimmedMessage}`);
            // Broadcast the movement command to all clients.
            wss.clients.forEach((client) => {
                if (client.readyState === WebSocket.OPEN) {
                    client.send(trimmedMessage);
                }
            });
            
            // Map and forward the command to the Arduino.
            const vCommand = getVCommand(trimmedMessage);
            if (!vCommand) {
                console.error(`Invalid movement command: ${trimmedMessage}`);
                return;
            }
            setTimeout(() => {
                arduinoPort.write(`${vCommand}\n`, 'utf8', (err) => {
                    if (err) {
                        console.error(`Error writing to Arduino: ${err}`);
                    } else {
                        console.log(`Forwarded movement command to Arduino (after 50ms delay): ${vCommand}`);
                        arduinoPort.drain(() => {
                            console.log(`Drain complete for command: ${vCommand}`);
                        });
                    }
                });
            }, 50);


        } else {
            console.error(`Received unknown WebSocket message: ${trimmedMessage}`);
        }
    });

    ws.on('close', () => {
        console.log(`WebSocket Client Disconnected: ${clientIP}`);
    });

    ws.on('error', (err) => {
        console.error(`WebSocket Error: ${err.message}`);
    });
});
