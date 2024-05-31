import React, { useState, useEffect, useCallback } from 'react';

const Map = () => {
    const [wsMapGui, setWsMapGui] = useState<WebSocket | null>(null);
    const [isConnecting, setIsConnecting] = useState(false); // Track if a connection attempt is underway
    const [mapImageUrl, setMapImageUrl] = useState('');
    const [goToLocationMsg, setGoToLocationMsg] = useState('');
    const [poseData, setPoseData] = useState({ position: {}, orientation: {} });
    const [keyLocations, setKeySpecialLocations] = useState({});

    // Helper function to initialize WebSocket connection
    const connectMapGui = useCallback(() => {
        if (!wsMapGui || wsMapGui.readyState === WebSocket.CLOSED) {
            if (!isConnecting) {
                setIsConnecting(true);
                const websocket = new WebSocket('ws://localhost:8080/map');
                websocket.onmessage = handleWebSocketMessage;
                websocket.onopen = () => {
                    console.log("Connected to MapGui WebSocket");
                    setIsConnecting(false);
                };
                websocket.onerror = error => {
                    console.error("WebSocket error:", error);
                    setIsConnecting(false);
                };
                websocket.onclose = event => {
                    console.log("WebSocket connection closed with code:", event.code, "and reason:", event.reason);
                    setWsMapGui(null);
                    setIsConnecting(false);
                };
                setWsMapGui(websocket);
            }
        } else {
            console.log("WebSocket is already open.");
        }
    }, [wsMapGui, isConnecting]);

    // Handles all WebSocket messages
    const handleWebSocketMessage = (event: MessageEvent) => {
        const data = JSON.parse(event.data);
        if (data.mapImageUrl) {
            setMapImageUrl(data.mapImageUrl);
        } else if (data.position && data.orientation) {
            setPoseData({ position: data.position, orientation: data.orientation });
        } else if (data.keyLocations) {
            setKeyLocations(data.keyLocations);
        } else if (data.success !== undefined) {
            setGoToLocationMsg(`${data.success ? 'Success: ' : 'Failure: '} ${data.message}`);
        } else {
            console.log("Received unhandled data type:", data);
        }
    };

    // Sends a request to fetch map data
    const fetchMapData = () => {
        if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
            wsMapGui.send(JSON.stringify({ action: 'fetchMapData' }));
        } else {
            console.log("WebSocket is not open. Attempting to reconnect...");
            connectMapGui();
        }
    };

    // Sends various other requests to the WebSocket
    const sendWebSocketRequest = (action: string, additionalData?: object) => {
        if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
            wsMapGui.send(JSON.stringify({ action, ...additionalData }));
        } else {
            console.log("WebSocket is not open. Attempting to reconnect...");
            connectMapGui();
        }
    };

    // Clean up WebSocket connection
    useEffect(() => {
        return () => {
            if (wsMapGui) wsMapGui.close();
        };
    }, [wsMapGui]);

    return (
        <div className="map-container">
            <button onClick={fetchMapData}>Fetch Map Data</button>
            <button onClick={() => sendWebSocketRequest('updateRobotLocation')}>Update Robot Location</button>
            <button onClick={() => sendWebSocketRequest('fetchRobotPose')}>Fetch Robot Pose</button>
            <button onClick={() => sendWebSocketRequest('fetchKeyLocations')}>Fetch Key Locations</button>
            {mapImageUrl && <img src={mapImageUrl} alt="Map" />}
            {goToLocationMsg && <div className="alert">{goToLocationMsg}</div>}
            <div className="pose-data">
                <div>Position - X: {poseData.position.x}, Y: {poseData.position.y}, Z: {poseData.position.z}</div>
                <div>Orientation - X: {poseData.orientation.x}, Y: {poseData.orientation.y}, Z: {poseData.orientation.z}, W: {poseData.orientation.w}</div>
            </div>
            <div className="key-locations">
                {Object.keys(keyLocations).map(key => (
                    <div key={key}>{keyLocations[key].name}: X={keyLocations[key].x}, Y={keyLocations[key].y}</div>
                ))}
            </div>
        </div>
    );
};

export default Map;
