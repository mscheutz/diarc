import React, { useState, useEffect, useCallback, useRef } from 'react';

import { Button } from "../Button";
import useWebSocket from 'react-use-websocket';
import ConnectionIndicator from './ConnectionIndicator';

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

const position2String = (pose: Pose | null) => {
    if (pose === null)
        return "Position not known";

    return `Position — X: ${pose.position.x}, Y: ${pose.position.y}, ` +
        `Z: ${pose.position.z}`;
};

const orientation2String = (pose: Pose | null) => {
    if (pose === null)
        return "Orientation not known";

    return `Orientation — X: ${pose.orientation.x}, Y: ${pose.orientation.y}, ` +
        `Z: ${pose.orientation.z}, W: ${pose.orientation.w}`
};

const keyLocations2Elements = (keyLocations) => {
    return Object.keys(keyLocations).map(key => (
        <p key={key}>
            {keyLocations[key].name}: X={keyLocations[key].x}, Y={keyLocations[key].y}
        </p>
    ))
};

const MapViewer = () => {
    // State //
    const [poseData, setPoseData] = useState<Pose | null>(null);
    const [goToLocationMsg, setGoToLocationMsg] = useState<string>("");
    const [keyLocations, setKeyLocations] = useState({});
    const [mapImageUrl, setMapImageUrl] = useState<string>("");

    // Set up Websocket
    const { sendMessage, lastMessage, readyState } =
        useWebSocket("ws://localhost:8080/map");

    // const [wsMapGui, setWsMapGui] = useState<WebSocket | null>(null);
    // const [isConnecting, setIsConnecting] = useState(false); // Track if a connection attempt is underway

    // Handle received messages
    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);
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
    },
        [lastMessage]
    )

    // Sends a request to fetch map data
    const fetchMapData = () =>
        sendMessage(JSON.stringify({ action: "fetchMapData" }));

    // Sends various other requests to the WebSocket
    const sendWebSocketRequest = (action: string, additionalData?: object) =>
        sendMessage(JSON.stringify({ action, ...additionalData }));

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
        context.strokeStyle = "#d1dbe3";
        context.strokeRect(1, 1, canvas.width - 1, canvas.height - 1);

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
        <div className="map-container h-full w-full flex flex-col gap-5 outline
                        outline-1 outline-[#d1dbe3] items-center p-5 rounded-md">
            {/* Button menu */}
            <div className="flex flex-row justify-center gap-2">
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

            <canvas ref={canvasRef} className='Map w-11/12' />

            {goToLocationMsg && <div className="alert">{goToLocationMsg}</div>}

            <div className="pose-data">
                <div>{position2String(poseData)}</div>
                <div>{orientation2String(poseData)}</div>
            </div>

            <div className="key-locations">
                {keyLocations2Elements(keyLocations)}
            </div>

            <ConnectionIndicator readyState={readyState} />
        </div>
    );
};

export default MapViewer;
