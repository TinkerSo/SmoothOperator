import React, { useState } from 'react';
import './InteractionPage.css';

const InteractionPage = () => {
  const [screen, setScreen] = useState('authentication');

  return (
    <div className="interaction-container">
      {screen === 'authentication' && (
        <div className="screen-box">
          <h1 className="title">Welcome to SmoothOperator™</h1>
          <p className="subtitle">Authenticate to Proceed</p>
          <p className="instruction">Please scan your ticket or enter a code to begin.</p>
          <div className="actions">
            <button className="action-button" onClick={() => setScreen('scanning')}>Scan Ticket</button>
            <button className="action-button" onClick={() => setScreen('manualEntry')}>Enter Code</button>
          </div>
        </div>
      )}

      {screen === 'scanning' && (
        <div className="screen-box">
          <h1 className="title">Scanning Ticket...</h1>
          <p className="instruction">Place your ticket under the scanner.</p>
          <button className="back-button" onClick={() => setScreen('authentication')}>Back</button>
        </div>
      )}

      {screen === 'manualEntry' && (
        <div className="screen-box">
          <h1 className="title">Manual Code Entry</h1>
          <input type="text" placeholder="Enter code here..." className="input-field" />
          <div className="actions">
            <button className="action-button">Submit</button>
            <button className="back-button" onClick={() => setScreen('authentication')}>Back</button>
          </div>
        </div>
      )}
    </div>
  );
};

export default InteractionPage;

