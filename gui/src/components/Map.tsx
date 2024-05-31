import React, { useState, useEffect } from 'react';

const Map = () => {
  const [wsMapGui, setWsMapGui] = useState<WebSocket | null>(null);
  const [mapData, setMapData] = useState<string>('');

  // Establishes a WebSocket connection to the map server
  const connectMapGui = () => {
    if (!wsMapGui || wsMapGui.readyState === WebSocket.CLOSED) {
      const websocket = new WebSocket('ws://localhost:8080/map');
      websocket.onmessage = (event) => {
        showMapData(event.data);
      };
      websocket.onopen = () => {
        console.log("Connected to MapGui WebSocket");
      };
      websocket.onerror = (error) => {
        console.error("WebSocket error:", error);
      };
      websocket.onclose = (event) => {
        console.log("WebSocket connection closed with code:", event.code, "and reason:", event.reason);
        setWsMapGui(null);
      };
      setWsMapGui(websocket);
    } else {
      console.log("WebSocket is already open.");
    }
  };

  // Fetch map data via WebSocket
  const fetchMapData = () => {
    if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
      wsMapGui.send(JSON.stringify({ action: 'fetchMapData' }));
    } else {
      console.log("WebSocket is not open. Attempting to reconnect...");
      connectMapGui();
      setTimeout(fetchMapData, 500); // Retry after delay
    }
  };

  // Displays map data or error based on server response
  const showMapData = (message) => {
    const data = JSON.parse(message);
    if (data.mapImageUrl) {
      setMapData(`<img src='${data.mapImageUrl}' alt='Map Image'/>`);
    } else {
      setMapData(`Error: ${data.error}`);
    }
  };

  // Clean up WebSocket connection
  useEffect(() => {
    return () => {
      if (wsMapGui) {
        wsMapGui.close();
      }
    };
  }, [wsMapGui]);

  return (
    <div>
      <button onClick={connectMapGui}>Connect to Map Service</button>
      <button onClick={fetchMapData}>Fetch Map Data</button>
      <div dangerouslySetInnerHTML={{ __html: mapData }} />
    </div>
  );
};

export default Map;
