// src/LandingPage.js
import React from 'react';
import { useNavigate } from 'react-router-dom';
import Face from './Face';
import './LandingPage.css';

const LandingPage = () => {
  const navigate = useNavigate();

  const handleGetStarted = () => {
    navigate('/interaction');
  };

  return (
    <div className="landing-container">
      <div className="face-section">
        <Face />
      </div>
      <div className="button-section">
        <button onClick={handleGetStarted}>Get Started</button>
      </div>
    </div>
  );
};

export default LandingPage;

