import React, { useState, useRef, useEffect } from 'react';
import {
  View,
  Text,
  TouchableOpacity,
  TextInput,
  Alert,
  StyleSheet,
  SafeAreaView,
  StatusBar,
  Dimensions,
} from 'react-native';

export default function GamepadWithAuth() {
  const ws = useRef(null);
  const [authenticated, setAuthenticated] = useState(false);
  const [passcode, setPasscode] = useState('');
  const [attempts, setAttempts] = useState(0);
  // New state for speed selection (default is Low, "L")
  const [currentSpeed, setCurrentSpeed] = useState('L');

  // Update these to your server's addresses:
  // const SERVER_IP = 'ws://192.168.1.5:3000'; // Jetson on Netgear
  // const HTTP_SERVER = 'http://128.197.53.43:3000'; // Jetson on Ethernet
  // const WS_SERVER = 'ws://128.197.53.43:3000'; // Jetson on Ethernet
  // const HTTP_SERVER = 'http://10.192.31.229:3000'; // Jetson on BU Guest
  // const WS_SERVER = 'ws://10.192.31.229:3000'; // Jetson on BU Guest
  const HTTP_SERVER = 'http://10.193.24.226:3000'; // Jetson on BU Guest SO2
  const WS_SERVER = 'ws://10.193.24.226:3000'; // Jetson on BU Guest SO2
  
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
        // Handle incoming messages as needed.
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

  // If not authenticated, show the passcode screen.
  if (!authenticated) {
    return (
      <SafeAreaView style={styles.safeArea}>
        <StatusBar barStyle="light-content" backgroundColor="#1E2C3A" />
        <View style={styles.passcodeContainer}>
          <View style={styles.passcodeCard}>
            <Text style={styles.passcodeTitle}>SmoothOperator Controller</Text>
            <Text style={styles.passcodeInstructions}>
              Please enter your 4-digit passcode to continue
            </Text>
            <TextInput
              style={styles.passcodeInput}
              value={passcode}
              onChangeText={handlePasscodeChange}
              keyboardType="number-pad"
              secureTextEntry={true}
              maxLength={4}
              placeholder="****"
              placeholderTextColor="#8C9BAA"
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
        </View>
      </SafeAreaView>
    );
  }

  // --------------------- Remote Control / Gamepad ---------------------
  // Function to handle directional button press
  const handlePressIn = (command) => {
    sendCommand(command);
  };

  // Function to handle directional button release (sending a stop command)
  const handlePressOut = () => {
    sendCommand('x');
  };

  // Function to handle speed change. Updates state and sends the speed command.
  const handleSpeedChange = (speed) => {
    setCurrentSpeed(speed);
    sendCommand(speed);
  };

  return (
    <SafeAreaView style={styles.safeArea}>
      <StatusBar barStyle="light-content" backgroundColor="#1E2C3A" />
      <View style={styles.container}>
        <View style={styles.header}>
          <Text style={styles.headerText}>SmoothOperator Controller</Text>
        </View>
        
        {/* Speed Buttons at the Top */}
        <View style={styles.speedButtonsContainer}>
          <Text style={styles.sectionTitle}>Speed</Text>
          <View style={styles.speedButtonsRow}>
            <TouchableOpacity
              style={[
                styles.speedButton,
                currentSpeed === 'L' && styles.activeSpeedButton,
              ]}
              onPress={() => handleSpeedChange('L')}
            >
              <Text style={[styles.speedButtonText, currentSpeed === 'L' && styles.activeSpeedText]}>Low</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[
                styles.speedButton,
                currentSpeed === 'M' && styles.activeSpeedButton,
              ]}
              onPress={() => handleSpeedChange('M')}
            >
              <Text style={[styles.speedButtonText, currentSpeed === 'M' && styles.activeSpeedText]}>Medium</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={[
                styles.speedButton,
                currentSpeed === 'H' && styles.activeSpeedButton,
              ]}
              onPress={() => handleSpeedChange('H')}
            >
              <Text style={[styles.speedButtonText, currentSpeed === 'H' && styles.activeSpeedText]}>High</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* Movement Section */}
        <View style={styles.sectionContainer}>
          <Text style={styles.sectionTitle}>Movement Controls</Text>
          
          {/* Directional Controls */}
          <View style={styles.controlsContainer}>
            <TouchableOpacity
              style={styles.button}
              onPressIn={() => handlePressIn('wr')}
              onPressOut={handlePressOut}
            >
              <Text style={styles.directionText}>FORWARD</Text>
            </TouchableOpacity>
            <View style={styles.row}>
              <TouchableOpacity
                style={styles.button}
                onPressIn={() => handlePressIn('ar')}
                onPressOut={handlePressOut}
              >
                <Text style={styles.directionText}>LEFT</Text>
              </TouchableOpacity>
              <View style={styles.spacer} />
              <TouchableOpacity
                style={styles.button}
                onPressIn={() => handlePressIn('dr')}
                onPressOut={handlePressOut}
              >
                <Text style={styles.directionText}>RIGHT</Text>
              </TouchableOpacity>
            </View>
            <TouchableOpacity
              style={styles.button}
              onPressIn={() => handlePressIn('sr')}
              onPressOut={handlePressOut}
            >
              <Text style={styles.directionText}>BACK</Text>
            </TouchableOpacity>
            <TouchableOpacity style={styles.stopButton} onPress={() => sendCommand('x')}>
              <Text style={styles.stopButtonText}>STOP</Text>
            </TouchableOpacity>
          </View>
        </View>

        {/* --------------------- Lift Controls --------------------- */}
        <View style={styles.sectionContainer}>
          <Text style={styles.sectionTitle}>Lift Controls</Text>
          <View style={styles.liftContainer}>
            <TouchableOpacity
              style={styles.liftButton}
              onPressIn={() => sendCommand('+')}
              onPressOut={() => sendCommand('=')}
            >
              <Text style={styles.liftButtonText}>Lift Up</Text>
            </TouchableOpacity>
            <TouchableOpacity
              style={styles.liftButton}
              onPressIn={() => sendCommand('-')}
              onPressOut={() => sendCommand('=')}
            >
              <Text style={styles.liftButtonText}>Lift Down</Text>
            </TouchableOpacity>
          </View>
        </View>
      </View>
    </SafeAreaView>
  );
}

const { width } = Dimensions.get('window');
const buttonSize = Math.min(80, width * 0.18);

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: '#1E2C3A',
  },
  container: {
    flex: 1,
    backgroundColor: '#F2F7FA',
    padding: 16,
  },
  header: {
    backgroundColor: '#1E2C3A',
    paddingVertical: 16,
    marginHorizontal: -16,
    marginTop: -16,
    marginBottom: 16,
    alignItems: 'center',
  },
  headerText: {
    color: '#FFFFFF',
    fontSize: 20,
    fontWeight: 'bold',
  },
  sectionContainer: {
    backgroundColor: '#FFFFFF',
    borderRadius: 12,
    padding: 16,
    marginBottom: 16,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 3,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: '600',
    color: '#1E2C3A',
    marginBottom: 12,
  },
  // Speed buttons container at the top
  speedButtonsContainer: {
    backgroundColor: '#FFFFFF',
    borderRadius: 12,
    padding: 16,
    marginBottom: 16,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 3,
  },
  speedButtonsRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  speedButton: {
    flex: 1,
    backgroundColor: '#F2F7FA',
    paddingHorizontal: 8,
    paddingVertical: 12,
    borderRadius: 8,
    marginHorizontal: 4,
    alignItems: 'center',
    borderWidth: 1,
    borderColor: '#DFE6ED',
  },
  activeSpeedButton: {
    backgroundColor: '#3498DB',
    borderColor: '#3498DB',
  },
  speedButtonText: {
    color: '#4A5C6A',
    fontSize: 16,
    fontWeight: '600',
  },
  activeSpeedText: {
    color: '#FFFFFF',
  },
  // Directional buttons and gamepad styles
  controlsContainer: {
    alignItems: 'center',
  },
  row: {
    flexDirection: 'row',
    marginVertical: 8,
    alignItems: 'center',
  },
  spacer: {
    width: buttonSize * 1.6,
  },
  button: {
    width: buttonSize * 1.6,
    height: buttonSize,
    backgroundColor: '#3498DB',
    borderRadius: 12,
    justifyContent: 'center',
    alignItems: 'center',
    margin: 8,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.2,
    shadowRadius: 3,
    elevation: 4,
  },
  directionText: {
    fontSize: 12,
    color: 'white',
    fontWeight: 'bold',
  },
  stopButton: {
    width: buttonSize * 1.6,
    height: buttonSize * 0.8,
    backgroundColor: '#E74C3C',
    borderRadius: 12,
    justifyContent: 'center',
    alignItems: 'center',
    marginTop: 16,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.2,
    shadowRadius: 3,
    elevation: 4,
  },
  buttonText: {
    fontSize: 24,
    color: 'white',
    fontWeight: 'bold',
  },
  stopButtonText: {
    fontSize: 16,
    color: 'white',
    fontWeight: 'bold',
  },
  // Lift controls container
  liftContainer: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  liftButton: {
    flex: 1,
    backgroundColor: '#9B59B6',
    paddingVertical: 14,
    borderRadius: 8,
    marginHorizontal: 8,
    alignItems: 'center',
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.2,
    shadowRadius: 3,
    elevation: 4,
  },
  liftButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: 'bold',
  },
  // Passcode screen styles
  passcodeContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 20,
    backgroundColor: '#1E2C3A',
  },
  passcodeCard: {
    backgroundColor: '#FFFFFF',
    borderRadius: 16,
    padding: 24,
    width: '90%',
    alignItems: 'center',
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.2,
    shadowRadius: 8,
    elevation: 5,
  },
  passcodeTitle: {
    marginBottom: 16,
    textAlign: 'center',
    fontSize: 24,
    fontWeight: 'bold',
    color: '#1E2C3A',
  },
  passcodeInstructions: {
    marginBottom: 24,
    textAlign: 'center',
    fontSize: 16,
    color: '#4A5C6A',
  },
  passcodeInput: {
    width: '80%',
    height: 56,
    borderWidth: 1,
    borderColor: '#DFE6ED',
    borderRadius: 12,
    paddingHorizontal: 16,
    fontSize: 20,
    marginBottom: 16,
    backgroundColor: '#F6F9FC',
    textAlign: 'center',
  },
  passcodeHints: {
    flexDirection: 'row',
    justifyContent: 'center',
    marginTop: 8,
  },
  passcodeHint: {
    fontSize: 24,
    letterSpacing: 16,
    color: '#3498DB',
  },
});