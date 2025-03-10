// src/InteractionPage.js
import React, { useState } from 'react';

const InteractionPage = () => {
  const [screen, setScreen] = useState('authentication');

  return (
    <div style={styles.container}>
      {screen === 'authentication' && (
        <div style={styles.fullScreenBox}>
          <h1>Authenticate to Proceed</h1>
          <p>Please scan your ticket or enter a code to begin.</p>
          <div style={styles.actions}>
            <button style={styles.actionButton} onClick={() => setScreen('scanning')}>
              Scan Ticket
            </button>
            <button style={styles.actionButton} onClick={() => setScreen('manualEntry')}>
              Enter Code
            </button>
          </div>
        </div>
      )}

      {screen === 'scanning' && (
        <div style={styles.fullScreenBox}>
          <h1>Scanning Ticket...</h1>
          <p>Place your ticket under the scanner.</p>
          <button style={styles.backButton} onClick={() => setScreen('authentication')}>
            Back
          </button>
        </div>
      )}

      {screen === 'manualEntry' && (
        <div style={styles.fullScreenBox}>
          <h1>Manual Code Entry</h1>
          <input type="text" placeholder="Enter code here..." style={styles.input} />
          <div style={styles.actions}>
            <button style={styles.actionButton}>Submit</button>
            <button style={styles.backButton} onClick={() => setScreen('authentication')}>
              Back
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

const styles = {
  container: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
    height: '100vh',
    width: '100vw',
    backgroundColor: '#f4f4f4',
    textAlign: 'center',
  },
  fullScreenBox: {
    width: '80%',
    maxWidth: '900px',
    padding: '40px',
    backgroundColor: '#fff',
    borderRadius: '12px',
    boxShadow: '0 4px 10px rgba(0, 0, 0, 0.1)',
  },
  actions: {
    marginTop: '30px',
    display: 'flex',
    justifyContent: 'center',
    gap: '30px',
  },
  actionButton: {
    padding: '15px 25px',
    fontSize: '20px',
    backgroundColor: '#28a745',
    color: 'white',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
  },
  backButton: {
    marginTop: '20px',
    padding: '15px 25px',
    fontSize: '18px',
    backgroundColor: '#dc3545',
    color: 'white',
    border: 'none',
    borderRadius: '8px',
    cursor: 'pointer',
  },
  input: {
    marginTop: '20px',
    padding: '15px',
    width: '60%',
    fontSize: '18px',
    border: '1px solid #ccc',
    borderRadius: '8px',
  },
};

export default InteractionPage;

