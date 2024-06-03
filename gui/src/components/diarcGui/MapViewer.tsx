import React, { useState, useEffect, useCallback, useRef } from 'react';

import { Button } from "../Button";

type Location = {
    x: number,
    y: number
}

type Position = {
    x: number,
    y: number,
    z: number
};

type Orientation = {
    x: number,
    y: number,
    z: number,
    w: number
};

type Pose = {
    position: Position,
    orientation: Orientation
};

export { Location, Position, Orientation, Pose };

const getPositionString = (pose: Pose | null) => {
    if (pose === null)
        return "Position not known";

    return `Position — X: ${pose.position.x}, Y: ${pose.position.y}, ` +
        `Z: ${pose.position.z}`;
};

const getOrientationString = (pose: Pose | null) => {
    if (pose === null)
        return "Orientation not known";

    return `Orientation — X: ${pose.orientation.x}, Y: ${pose.orientation.y}, ` +
        `Z: ${pose.orientation.z}, W: ${pose.orientation.w}`
};

const getKeyLocations = (keyLocations) => {
    return Object.keys(keyLocations).map(key => (
        <p key={key}>
            {keyLocations[key].name}: X={keyLocations[key].x}, Y={keyLocations[key].y}
        </p>
    ))
};

const MapViewer = () => {
    const [wsMapGui, setWsMapGui] = useState<WebSocket | null>(null);
    const [isConnecting, setIsConnecting] = useState(false); // Track if a connection attempt is underway
    const [mapImageUrl, setMapImageUrl] = useState<string>("");
    const [goToLocationMsg, setGoToLocationMsg] = useState<string>("");
    const [poseData, setPoseData] = useState<Pose | null>(null);
    const [keyLocations, setKeyLocations] = useState({});

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
            console.warn("Received unhandled data type:", data);
        }
    };

    // Sends a request to fetch map data
    const fetchMapData = () => {
        if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
            wsMapGui.send(JSON.stringify({ action: 'fetchMapData' }));
        } else {
            console.warn("WebSocket is not open. Attempting to reconnect...");
            connectMapGui();
        }
    };

    // Sends various other requests to the WebSocket
    const sendWebSocketRequest = (action: string, additionalData?: object) => {
        if (wsMapGui && wsMapGui.readyState === WebSocket.OPEN) {
            wsMapGui.send(JSON.stringify({ action, ...additionalData }));
        } else {
            console.warn("WebSocket is not open. Attempting to reconnect...");
            connectMapGui();
        }
    };

    // Clean up WebSocket connection
    useEffect(() => {
        return () => {
            if (wsMapGui) wsMapGui.close();
        };
    }, [wsMapGui]);

    // Try to open a web socket connection on load
    connectMapGui();

    // Map drawing
    const canvasRef = useRef(null);
    useEffect(() => {
        const canvas: HTMLCanvasElement = canvasRef.current!;
        const context = canvas.getContext("2d")!;

        // Make it higher resolution
        const pixelRatio = 2;
        canvas.width *= pixelRatio;
        canvas.height *= pixelRatio;

        // Background
        context.fillStyle = "#e0e0e0";
        context.fillRect(0, 0, canvas.width, canvas.height);

        // Placeholder
        if (mapImageUrl === "") {
            context.fillStyle = "#000000";
            context.textBaseline = "middle";
            context.textAlign = "center";
            context.font = "40px Ubuntu";
            context.fillText("Map not loaded",
                canvas.width / 2, canvas.height / 2);
        }
        // Map
        else {
            const image = document.createElement("img");
            image.src = mapImageUrl;
            image.onload = () => {
                const canvasAspectRatio = canvas.width / canvas.height;
                const imageAspectRatio = image.width / image.height;
                if (canvasAspectRatio >= imageAspectRatio) {
                    // Scale image so that its height equals the canvas height
                    const newImageWidth = canvas.height / image.height * image.width;
                    const xPad = (canvas.width - newImageWidth) / 2;
                    context.drawImage(image, xPad, 0, newImageWidth, canvas.height);
                } else {
                    // Scale image so that its width equals the canvas width
                    const newImageHeight = canvas.width / image.width * image.height;
                    const yPad = (canvas.height - newImageHeight) / 2;
                    context.drawImage(image, 0, yPad, image.width, newImageHeight);
                }
            }
        }
    },
        [mapImageUrl]
    );

    return (
        <div className="map-container h-full w-full flex flex-col gap-5">
            {/* Button menu */}
            <div className="flex flex-row justify-center gap-2 mt-2">
                <Button
                    onClick={fetchMapData}>
                    Fetch Map Data
                </Button>
                <Button
                    onClick={() => sendWebSocketRequest('updateRobotLocation')}>
                    Update Robot Location
                </Button>
                <Button
                    onClick={() => sendWebSocketRequest('fetchRobotPose')}>
                    Fetch Robot Pose
                </Button>
                <Button
                    onClick={() => sendWebSocketRequest('fetchKeyLocations')}>
                    Fetch Key Locations
                </Button>
            </div>

            <canvas ref={canvasRef} className='Map' />

            {goToLocationMsg && <div className="alert">{goToLocationMsg}</div>}

            <div className="pose-data">
                <div>{getPositionString(poseData)}</div>
                <div>{getOrientationString(poseData)}</div>
            </div>

            <div className="key-locations">
                {getKeyLocations(keyLocations)}
            </div>
        </div>
    );
};

export default MapViewer;
