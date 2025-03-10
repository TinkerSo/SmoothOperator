// src/Face.js
import React, { useState, useEffect, useRef } from 'react';
import './Face.css';

const Face = () => {
  const faceRef = useRef(null);
  const [offset, setOffset] = useState({ x: 0, y: 0 });
  const [blinking, setBlinking] = useState(false);

  const handleMouseMove = (event) => {
    if (!faceRef.current) return;
    const rect = faceRef.current.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    const deltaX = event.clientX - centerX;
    const deltaY = event.clientY - centerY;
    const angle = Math.atan2(deltaY, deltaX);
    const maxOffset = 10;
    const distance = Math.min(maxOffset, Math.hypot(deltaX, deltaY) / 20);
    const offsetX = Math.cos(angle) * distance;
    const offsetY = Math.sin(angle) * distance;
    setOffset({ x: offsetX, y: offsetY });
  };

  useEffect(() => {
    const faceNode = faceRef.current;
    if (faceNode) {
      faceNode.addEventListener('mousemove', handleMouseMove);
    }
    return () => {
      if (faceNode) {
        faceNode.removeEventListener('mousemove', handleMouseMove);
      }
    };
  }, []);

  useEffect(() => {
    let blinkTimeout;
    const triggerBlink = () => {
      setBlinking(true);
      setTimeout(() => {
        setBlinking(false);
        const nextBlink = Math.random() * (10000 - 3000) + 3000;
        blinkTimeout = setTimeout(triggerBlink, nextBlink);
      }, 200);
    };
    const initialDelay = Math.random() * (10000 - 3000) + 3000;
    blinkTimeout = setTimeout(triggerBlink, initialDelay);
    return () => {
      clearTimeout(blinkTimeout);
    };
  }, []);

  const eyeStyle = {
    transform: `translate(${offset.x}px, ${offset.y}px) ${blinking ? 'scaleY(0.1)' : ''}`,
  };

  return (
    <div className="face" ref={faceRef}>
      <div className="eye left-eye" style={eyeStyle}></div>
      <div className="eye right-eye" style={eyeStyle}></div>
      <div className="smile"></div>
    </div>
  );
};

export default Face;

