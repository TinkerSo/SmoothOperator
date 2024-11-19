const WebSocket = require('ws');
const keypress = require('keypress');

// Connect to the WebSocket server running on localhost
const ws = new WebSocket('ws://192.168.1.3:8080');

keypress(process.stdin); // Enable keypress event on stdin

ws.on('open', () => {
    console.log('Connected to server');
    console.log('Press "w", "a", "s", "d", "x" to send movement commands. Press "z" to exit.');

    // Listen for keypress events
    process.stdin.on('keypress', (char, key) => {
        if (key && key.name === 'z') {
            ws.close(); // Close the connection on 'x'
            process.stdin.pause(); // Stop listening for keypresses
            return;
        }

        if (['w', 'a', 's', 'd', 'x'].includes(char)) {
            ws.send(char);
            //console.log(`Sent command: ${char}`);
        } else {
            //console.log('Invalid command. Please press "w", "a", "s", "d" or "x".');
        }
    });

    process.stdin.setRawMode(true); // Enable raw mode to capture keypresses
    process.stdin.resume(); // Resume stdin to start receiving input
});

ws.on('close', () => {
    console.log('Disconnected from server');
});
