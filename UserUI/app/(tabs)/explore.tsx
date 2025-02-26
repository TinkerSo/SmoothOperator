import React, { useEffect, useRef } from 'react';
import { View, Text, TouchableOpacity, StyleSheet } from 'react-native';

export default function GamepadControls() {
  const ws = useRef(null);
  const intervalRef = useRef(null);
  // Use your desired server IP address here
  const SERVER_IP = 'ws://10.192.31.229:3000'; // Jetson on BU Guest
  // const SERVER_IP = 'ws://192.168.1.5:3000'; // Jetson on Netgear

  useEffect(() => {
    let retryCount = 0;
    const maxRetries = 5;

    const connectWebSocket = () => {
      console.log(`Attempting to connect to WebSocket: ${SERVER_IP}`);
      ws.current = new WebSocket(SERVER_IP);

      ws.current.onopen = () => {
        console.log('✅ Connected to WebSocket server');
        retryCount = 0;
      };

      ws.current.onmessage = (event) => {
        console.log('📩 Message from server:', event.data);
      };

      ws.current.onerror = (error) => {
        console.error('❌ WebSocket error:', error);
      };

      ws.current.onclose = () => {
        console.log('🔴 Disconnected from WebSocket server');
        if (retryCount < maxRetries) {
          retryCount++;
          console.log(`🔄 Retrying connection in 3 seconds (Attempt ${retryCount}/${maxRetries})`);
          setTimeout(connectWebSocket, 3000);
        } else {
          console.error('🚨 WebSocket connection failed after multiple attempts.');
        }
      };
    };

    connectWebSocket();

    return () => {
      if (ws.current) ws.current.close();
    };
  }, []);

  const sendCommand = (command) => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      ws.current.send(command);
      console.log(`📤 Sent command: ${command}`);
    } else {
      console.error('⚠️ WebSocket is not connected.');
    }
  };

  const handlePressIn = (command) => {
    sendCommand(command);
  };

  const handlePressOut = () => {
    sendCommand('x'); // Send stop command when released
  };

  return (
    <View style={styles.container}>
      <TouchableOpacity
        style={styles.button}
        onPressIn={() => handlePressIn('w')}
        onPressOut={handlePressOut}
      >
        <Text style={styles.buttonText}>⬆️</Text>
      </TouchableOpacity>
      <View style={styles.row}>
        <TouchableOpacity
          style={styles.button}
          onPressIn={() => handlePressIn('a')}
          onPressOut={handlePressOut}
        >
          <Text style={styles.buttonText}>⬅️</Text>
        </TouchableOpacity>
        <View style={styles.spacer} />
        <TouchableOpacity
          style={styles.button}
          onPressIn={() => handlePressIn('d')}
          onPressOut={handlePressOut}
        >
          <Text style={styles.buttonText}>➡️</Text>
        </TouchableOpacity>
      </View>
      <TouchableOpacity
        style={styles.button}
        onPressIn={() => handlePressIn('s')}
        onPressOut={handlePressOut}
      >
        <Text style={styles.buttonText}>⬇️</Text>
      </TouchableOpacity>
      <TouchableOpacity style={styles.stopButton} onPress={() => sendCommand('x')}>
        <Text style={styles.buttonText}>🛑 Stop</Text>
      </TouchableOpacity>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: '#f0f0f0',
    padding: 20,
  },
  row: {
    flexDirection: 'row',
    marginVertical: 10,
    alignItems: 'center',
  },
  spacer: {
    width: 130,
  },
  button: {
    width: 80,
    height: 80,
    backgroundColor: '#007bff',
    borderRadius: 40,
    justifyContent: 'center',
    alignItems: 'center',
    margin: 10,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.25,
    shadowRadius: 3.84,
    elevation: 5,
  },
  stopButton: {
    width: 100,
    height: 80,
    backgroundColor: '#ff0000',
    borderRadius: 40,
    justifyContent: 'center',
    alignItems: 'center',
    marginTop: 20,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.25,
    shadowRadius: 3.84,
    elevation: 5,
  },
  buttonText: {
    fontSize: 24,
    color: 'white',
    fontWeight: 'bold',
  },
});
