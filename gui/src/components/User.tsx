import React, { useState, useEffect } from 'react';

const User = () => {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [messages, setMessages] = useState<string[]>([]);
  const [inputMessage, setInputMessage] = useState('');

  const connect = () => {
    const websocket = new WebSocket('ws://${process.env.REACT_APP_SERVER_ADDR}:8000/user');
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

  const disconnect = () => {
    if (ws) {
      ws.close();
      setWs(null);
    }
  };

  const sendMessage = () => {
    if (ws && ws.readyState === WebSocket.OPEN && inputMessage) {
      ws.send(JSON.stringify({ 'user': inputMessage }));
      setInputMessage('');
    }
  };

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
