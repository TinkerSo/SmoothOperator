import React, { useState, useRef, useEffect } from 'react';
import {
  View,
  Text,
  TouchableOpacity,
  TextInput,
  Alert,
  StyleSheet,
} from 'react-native';

export default function GamepadWithAuth() {
  const ws = useRef(null);
  const [authenticated, setAuthenticated] = useState(false);
  const [passcode, setPasscode] = useState('');
  const [attempts, setAttempts] = useState(0);
  
  // Update these to your server's addresses:
  // const SERVER_IP = 'ws://10.192.31.229:3000'; // Jetson on BU Guest
// const SERVER_IP = 'ws://192.168.1.5:3000'; // Jetson on Netgear
  const HTTP_SERVER = 'http://128.197.53.43:3000';
  const WS_SERVER = 'ws://128.197.53.43:3000'; // Jetson on Netgear
  const maxRetries = 5;
  const retryCountRef = useRef(0);

  // Establish the WebSocket connection
  useEffect(() => {
    const connectWebSocket = () => {
      console.log(`Connecting to WebSocket at ${WS_SERVER}`);
      ws.current = new WebSocket(WS_SERVER);

      ws.current.onopen = () => {
        console.log('✅ WebSocket connection opened');
        retryCountRef.current = 0;
      };

      ws.current.onmessage = (event) => {
        console.log('📩 Received:', event.data);
        // You can add handling of incoming messages here if needed.
      };

      ws.current.onerror = (error) => {
        console.error('❌ WebSocket error:', error);
      };

      ws.current.onclose = () => {
        console.log('🔴 WebSocket connection closed');
        if (retryCountRef.current < maxRetries) {
          retryCountRef.current++;
          console.log(
            `🔄 Retrying connection in 3 seconds (Attempt ${retryCountRef.current}/${maxRetries})`
          );
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

  // Function to send gamepad commands via WebSocket
  const sendCommand = (command) => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      ws.current.send(command);
      console.log(`📤 Sent command: ${command}`);
    } else {
      console.error('⚠️ WebSocket is not connected.');
    }
  };

  // --------------------- Passcode Screen ---------------------
  // When the user types 4 digits, send them to /api/authenticate.
  const handlePasscodeChange = (text) => {
    const cleanedText = text.replace(/[^0-9]/g, '').slice(0, 4);
    setPasscode(cleanedText);

    if (cleanedText.length === 4) {
      fetch(`${HTTP_SERVER}/api/authenticate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ passcode: cleanedText }),
      })
        .then((response) => {
          if (response.ok) {
            console.log('✅ Passcode authenticated');
            setAuthenticated(true);
            // Send a success message over WebSocket to notify the Python app.
            if (ws.current && ws.current.readyState === WebSocket.OPEN) {
              ws.current.send('AUTH_SUCCESS');
              console.log('📤 Sent AUTH_SUCCESS via WebSocket');
            }
          } else {
            return response.json().then((data) => {
              throw new Error(data.error || 'Incorrect passcode');
            });
          }
        })
        .catch((error) => {
          console.error(error);
          setAttempts((prev) => prev + 1);
          Alert.alert(
            'Incorrect Passcode',
            `Please try again. Attempt ${attempts + 1} of 3.`,
            [{ text: 'OK', onPress: () => setPasscode('') }]
          );
          if (attempts >= 2) {
            Alert.alert('Too Many Attempts', 'Please try again later.', [{ text: 'OK' }]);
          }
        });
    }
  };

  if (!authenticated) {
    return (
      <View style={styles.passcodeContainer}>
        <Text style={styles.passcodeTitle}>Good Enter Passcode</Text>
        <Text style={styles.passcodeInstructions}>
          Please enter your 4-digit connect passcode to continue.
        </Text>
        <TextInput
          style={styles.passcodeInput}
          value={passcode}
          onChangeText={handlePasscodeChange}
          keyboardType="number-pad"
          secureTextEntry={true}
          maxLength={4}
          placeholder="****"
          placeholderTextColor="#999"
        />
        <View style={styles.passcodeHints}>
          <Text style={styles.passcodeHint}>
            {passcode.length >= 1 ? '●' : '○'}
            {passcode.length >= 2 ? '●' : '○'}
            {passcode.length >= 3 ? '●' : '○'}
            {passcode.length >= 4 ? '●' : '○'}
          </Text>
        </View>
      </View>
    );
  }

  // --------------------- Gamepad Controls ---------------------
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
  passcodeContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 20,
    backgroundColor: '#f0f0f0',
  },
  passcodeTitle: {
    marginBottom: 20,
    textAlign: 'center',
    fontSize: 28,
  },
  passcodeInstructions: {
    marginBottom: 30,
    textAlign: 'center',
    fontSize: 18,
  },
  passcodeInput: {
    width: '80%',
    height: 50,
    borderWidth: 1,
    borderColor: '#ccc',
    borderRadius: 8,
    paddingHorizontal: 15,
    fontSize: 18,
    marginBottom: 20,
  },
  passcodeHints: {
    flexDirection: 'row',
    justifyContent: 'center',
    marginTop: 10,
  },
  passcodeHint: {
    fontSize: 24,
    letterSpacing: 10,
  },
});
