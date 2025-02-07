import React from 'react';
import { View, Text, TouchableOpacity, StyleSheet } from 'react-native';

export default function GamepadControls() {
  const SERVER_URL = 'http://10.193.228.60:3000'; // Replace localhost with your actual IP

  const sendCommand = async (command) => {
    try {
      console.log(`Sending command: ${command}`); // Log command before sending
      await fetch(`${SERVER_URL}/api/command`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ command }),
      });
    } catch (error) {
      console.error('Error sending command:', error);
    }
  };

  return (
    <View style={styles.container}>
      <TouchableOpacity
        style={styles.button}
        onPress={() => sendCommand('up')}
      >
        <Text style={styles.buttonText}>⬆️</Text>
      </TouchableOpacity>
      <View style={styles.row}>
        <TouchableOpacity
          style={styles.button}
          onPress={() => sendCommand('left')}
        >
          <Text style={styles.buttonText}>⬅️</Text>
        </TouchableOpacity>
        <View style={styles.spacer} />
        <TouchableOpacity
          style={styles.button}
          onPress={() => sendCommand('right')}
        >
          <Text style={styles.buttonText}>➡️</Text>
        </TouchableOpacity>
      </View>
      <TouchableOpacity
        style={styles.button}
        onPress={() => sendCommand('down')}
      >
        <Text style={styles.buttonText}>⬇️</Text>
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
    width: 130, // Adjust the space between left and right arrows
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
  buttonText: {
    fontSize: 24,
    color: 'white',
    fontWeight: 'bold',
  },
});
