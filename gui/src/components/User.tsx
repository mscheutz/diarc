import React, { useState, useEffect } from 'react';

const User = () => {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [messages, setMessages] = useState<string[]>([]);
  const [inputMessage, setInputMessage] = useState('');

  // Initialize WebSocket connection
  const connect = () => {
    const websocket = new WebSocket('ws://localhost:8080/user');
    websocket.onmessage = (event) => {
      setMessages((prevMessages) => [...prevMessages, event.data]);
    };
    websocket.onopen = () => {
      console.log("Connected to user WebSocket");
      setWs(websocket);
    };
    websocket.onclose = () => {
      console.log("User WebSocket is in disconnected state");
      setWs(null);
    };
    websocket.onerror = (error) => {
      console.log("WebSocket error:", error);
    };
  };

  // Disconnect WebSocket
  const disconnect = () => {
    if (ws) {
      ws.close();
      setWs(null);
    }
  };

  // Send a message
  const sendMessage = () => {
    if (ws && ws.readyState === WebSocket.OPEN && inputMessage) {
      ws.send(JSON.stringify({ 'user': inputMessage }));
      setInputMessage(''); // Clear input after sending
    }
  };

  // Cleanup WebSocket on component unmount
  useEffect(() => {
    return () => {
      if (ws) {
        ws.close();
      }
    };
  }, [ws]);

  return (
    <div>
      <button onClick={connect} disabled={ws !== null}>Start New Chat</button>
      <button onClick={disconnect} disabled={ws === null}>End Chat</button>
      <textarea
        value={inputMessage}
        onChange={(e) => setInputMessage(e.target.value)}
        placeholder="Write your message here..."
        required
      />
      <button onClick={sendMessage}>Send</button>
      <div>
        {messages.map((message, index) => (
          <p key={index}>{message}</p>
        ))}
      </div>
    </div>
  );
};

export default User;
