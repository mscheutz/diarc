import React, { useEffect, useCallback } from 'react';
import useWebSocket from 'react-use-websocket';

const VLAViewer = () => {
    const socketUrl = "ws://localhost:8000/ws";
    const { sendMessage, lastMessage, readyState } = useWebSocket(socketUrl, {
        onOpen: () => console.log("WebSocket connection opened"),
        onClose: () => console.log("WebSocket connection closed"),
        onError: (event) => console.error("WebSocket error:", event),
    });

    const sendWebSocketRequest = useCallback((command, action) => {
        const payload = { command, action };
        console.log("Sending WebSocket request:", payload); // Log the payload being sent
        sendMessage(JSON.stringify(payload));
    }, [sendMessage]);

    useEffect(() => {
        console.log("WebSocket readyState:", readyState); // Log WebSocket connection state
        if (lastMessage) {
            const data = JSON.parse(lastMessage.data);
            console.log("Received message from server:", data); // Log the message received from server
        }
    }, [lastMessage, readyState]);

    return (
        <div className="vla-container flex flex-col items-center gap-4 p-6">
            <button
                className="px-4 py-2 bg-green-500 text-white font-semibold rounded-lg hover:bg-green-600 transition duration-300"
                onClick={() => sendWebSocketRequest("start", "move near the mug")}
            >
                Start move near the mug
            </button>
            <button
                className="px-4 py-2 bg-blue-500 text-white font-semibold rounded-lg hover:bg-blue-600 transition duration-300"
                onClick={() => sendWebSocketRequest("start", "pick up the blue cube")}
            >
                Start Blue Cube
            </button>
            <button
                className="px-4 py-2 bg-red-500 text-white font-semibold rounded-lg hover:bg-red-600 transition duration-300"
                onClick={() => sendWebSocketRequest("start", "pick up the red cube")}
            >
                Start Red Cube
            </button>
        </div>
    );
};

export default VLAViewer;
