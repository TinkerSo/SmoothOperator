const { SerialPort, ReadlineParser } = require('serialport');
const { Readline } = require('@serialport/parser-readline');

// Adjust the port name to match your system
const port = new SerialPort('/dev/ttyACM0', {
  baudRate: 115200,
});

const parser = port.pipe(new Readline({ delimiter: '\n' }));

// Open the serial communication port
port.on('open', () => {
  console.log('Serial port open');
});

// Read the data from the serial port
parser.on('data', data =>{
  console.log('Received data:', data);
});

// Handle any errors that occur
port.on('error', function(err) {
  console.log('Error: ', err.message);
});

// Function to send data to the Arduino
function sendToArduino(data) {
  console.log('Sending to Arduino:', data);
  port.write(data, function(err) {
    if (err) {
      return console.log('Error on write: ', err.message);
    }
    console.log('message written');
  });
}

// Example sending data
setTimeout(() => {
  sendToArduino("Hello Arduino!\n");
}, 2000);
