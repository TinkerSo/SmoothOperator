import { Image, StyleSheet, Platform, TextInput, Alert } from 'react-native';
import { useState } from 'react';
import { HelloWave } from '@/components/HelloWave';
import ParallaxScrollView from '@/components/ParallaxScrollView';
import { ThemedText } from '@/components/ThemedText';
import { ThemedView } from '@/components/ThemedView';

export default function HomeScreen() {
  const [passcode, setPasscode] = useState('');
  const [authenticated, setAuthenticated] = useState(false);
  const [attempts, setAttempts] = useState(0);
  
  const handlePasscodeChange = (text) => {
    // Only allow digits and limit to 4 characters
    const cleanedText = text.replace(/[^0-9]/g, '').slice(0, 4);
    setPasscode(cleanedText);
    
    // When 4 digits are entered, authenticate via server
    if (cleanedText.length === 4) {
      fetch("http://<server-ip>:3000/api/authenticate", {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({ passcode: cleanedText })
      })
      .then(response => {
        if (response.ok) {
          setAuthenticated(true);
        } else {
          setAttempts(prev => prev + 1);
          response.json().then(data => {
            Alert.alert(
              "Incorrect Passcode", 
              `Please try again. Attempt ${attempts + 1} of 3.`,
              [{ text: "OK", onPress: () => setPasscode('') }]
            );
          });
          if (attempts >= 2) {
            Alert.alert(
              "Too Many Attempts", 
              "Please try again later.",
              [{ text: "OK" }]
            );
          }
        }
      })
      .catch(error => {
        console.error("Error authenticating:", error);
        Alert.alert("Error", "Failed to authenticate, please try again later.");
      });
    }
  };
  
  // If not authenticated, show passcode screen
  if (!authenticated) {
    return (
      <ThemedView style={styles.passcodeContainer}>
        <ThemedText type="title" style={styles.passcodeTitle}>Enter Passcode</ThemedText>
        <ThemedText style={styles.passcodeInstructions}>
          Please enter your 4-digit connect passcode to continue
        </ThemedText>
        
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
        
        <ThemedView style={styles.passcodeHints}>
          <ThemedText style={styles.passcodeHint}>
            {passcode.length >= 1 ? "●" : "○"}
            {passcode.length >= 2 ? "●" : "○"}
            {passcode.length >= 3 ? "●" : "○"}
            {passcode.length >= 4 ? "●" : "○"}
          </ThemedText>
        </ThemedView>
      </ThemedView>
    );
  }
  
  // If authenticated, show the main content
  return (
    <ParallaxScrollView
      headerBackgroundColor={{ light: '#A1CEDC', dark: '#1D3D47' }}
      headerImage={
        <Image
          source={require('@/assets/images/partial-react-logo.png')}
          style={styles.reactLogo}
        />
      }>
      <ThemedView style={styles.titleContainer}>
        <ThemedText type="title">Welcome!</ThemedText>
        <HelloWave />
      </ThemedView>
      <ThemedView style={styles.stepContainer}>
        <ThemedText type="subtitle">Step 1: Try it</ThemedText>
        <ThemedText>
         Edit <ThemedText type="defaultSemiBold">app/(tabs)/index.tsx</ThemedText> to see changes.
         Press{' '}
          <ThemedText type="defaultSemiBold">
            {Platform.select({
              ios: 'cmd + d',
              android: 'cmd + m',
              web: 'F12'
            })}
          </ThemedText>{' '}
         to open developer tools.
        </ThemedText>
      </ThemedView>
      <ThemedView style={styles.stepContainer}>
        <ThemedText type="subtitle">Step 2: Explore</ThemedText>
        <ThemedText>
         Tap the Explore tab to learn more about what's included in this starter app.
        </ThemedText>
      </ThemedView>
      <ThemedView style={styles.stepContainer}>
        <ThemedText type="subtitle">Step 3: Get a fresh start</ThemedText>
        <ThemedText>
         When you're ready, run{' '}
          <ThemedText type="defaultSemiBold">npm run reset-project</ThemedText> to get a fresh{' '}
          <ThemedText type="defaultSemiBold">app</ThemedText> directory. This will move the current{' '}
          <ThemedText type="defaultSemiBold">app</ThemedText> to{' '}
          <ThemedText type="defaultSemiBold">app-example</ThemedText>.
        </ThemedText>
      </ThemedView>
    </ParallaxScrollView>
  );
}

const styles = StyleSheet.create({
  titleContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  stepContainer: {
    gap: 8,
    marginBottom: 8,
  },
  reactLogo: {
    height: 178,
    width: 290,
    bottom: 0,
    left: 0,
    position: 'absolute',
  },
  passcodeContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 20,
  },
  passcodeTitle: {
    marginBottom: 20,
    textAlign: 'center',
  },
  passcodeInstructions: {
    marginBottom: 30,
    textAlign: 'center',
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
