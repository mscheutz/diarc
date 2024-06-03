import React, { useState, useEffect, useCallback, useRef } from 'react';

import { Button } from "../Button";
import useWebSocket, { ReadyState } from 'react-use-websocket';
import { faBan, faCheck, faQuestion, faSync } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
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
    );

    const handleNavigateToPoint = () => {
        const x = prompt("Enter X coordinate:");
        const y = prompt("Enter Y coordinate:");
        // Default values for orientation
        const quatX = 0;
        const quatY = 0;
        const quatZ = 0;
        const quatW = 1;

        if (x && y && !isNaN(Number(x)) && !isNaN(Number(y))) {
            const newX = Number(x);
            const newY = Number(y);

            // Set the pose data directly with new coordinates and default orientation
            const newPoseData = {
                position: { x: newX, y: newY, z: (poseData?.position.z || 0) }, // Maintain the Z value or default it to 0
                orientation: { x: quatX, y: quatY, z: quatZ, w: quatW }
            };

            setPoseData(newPoseData);
            sendWebSocketRequest('navigateToPoint', {
                x: newX,
                y: newY,
                quatX: quatX,
                quatY: quatY,
                quatZ: quatZ,
                quatW: quatW
            });
        } else {
            alert("Please enter valid numerical coordinates.");
        }
    }

    // Sends various other requests to the WebSocket
    const sendWebSocketRequest = useCallback((action: string, additionalData?: object) => {
        sendMessage(JSON.stringify({ action, ...additionalData }));
    }, [sendMessage]);

    // Map drawing
    const canvasRef = useRef(null);
    useEffect(() => {
        const canvas: HTMLCanvasElement = canvasRef.current!;
        const context = canvas.getContext("2d")!;

        // Make it higher resolution
        const pixelRatio = 1;
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
            const image = new Image();
            image.src = mapImageUrl;
            image.onload = () => {
                // Image scaling logic
                const scale = Math.min(canvas.width / image.width, canvas.height / image.height);
                const x = (canvas.width / 2) - (image.width / 2) * scale;
                const y = (canvas.height / 2) - (image.height / 2) * scale;

                context.drawImage(image, x, y, image.width * scale, image.height * scale);

                // Draw the robot position if it exists
                if (poseData) {
                    const robotX = x + poseData.position.x * scale;
                    const robotY = y + poseData.position.y * scale;
                    context.fillStyle = 'red';
                    context.beginPath();
                    // Adjust the size of the red dot to be appropriately visible
                    const radius = 10 * pixelRatio;
                    context.arc(robotX, robotY, radius, 0, 2 * Math.PI);
                    context.fill();
                }
            }
        }
    },
        [mapImageUrl, poseData]
    );

    return (
        <div className="map-container h-full w-full flex flex-col gap-3 outline
                        outline-1 outline-[#d1dbe3] items-center">
            {/* Button menu */}
            <div className="flex flex-row justify-center gap-2 mt-3">
                <Button
                    onClick={() => sendWebSocketRequest('fetchMapData')}>
                    Fetch Map Data
                </Button>
                <Button
                    onClick={handleNavigateToPoint}>
                    Navigate To Point
                </Button>
                <Button
                    onClick={() => sendWebSocketRequest('fetchRobotPose')}>
                    Show/Update Robot Pose
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
