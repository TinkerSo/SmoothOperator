// src/App.js
import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import InteractionPage from './InteractionPage';
import Face from './Face';

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<Face />} />
      </Routes>
    </Router>
  );
}

export default App;

