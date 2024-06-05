import React, { useState, useEffect, useCallback, useRef } from 'react';

import { Button } from "../Button";
import useWebSocket, { ReadyState } from 'react-use-websocket';
import { faBan, faCheck, faQuestion, faSync } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import ConnectionIndicator from './ConnectionIndicator';

type Location = {
    x: number;
    y: number;
};

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

type RobotPixelPosition = {
    x: number,
    y: number
};

type Pose = {
    position: Position,
    orientation: Orientation,
    robotPixelPosition: RobotPixelPosition
};

type KeyLocation = {
    name: string;
    x: number;
    y: number;
};

type KeyLocations = { [key: string]: KeyLocation };

export { Location, Position, Orientation, RobotPixelPosition, Pose, KeyLocation, KeyLocations };

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
    const [responseMsg, setResponseMsg] = useState<string>("");
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

        if (data.error) {
            alert(data.error); // Alert the user about the error
            console.error("Error received:", data.error);
            return; // Stop processing further as there was an error
        }

        if (data.mapImageUrl) {
            setMapImageUrl(data.mapImageUrl);
        }
        if (data.position && data.orientation) {
            setPoseData({ position: data.position, orientation: data.orientation, robotPixelPosition: data.robotPixelPosition });
        }
        if (data.keyLocations) {
            setKeyLocations(data.keyLocations);
        }
        if (data.message) {
            setResponseMsg(`${data.success ? 'Success: ' : 'Failure: '} ${data.message}`);
        }
    }, [lastMessage]);

    const handleGoToLocation = () => {
        const symbol = prompt("Enter the symbol for the location (e.g., location_0:location): ");
        if (symbol && symbol.trim() !== '') {
            sendWebSocketRequest('goToLocation', { locationSymbol: symbol.trim() });
        } else {
            alert("Please enter a valid Symbol for the location in the format of name:type");
        }
    };

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

    function mapCoordinates(x: number, y: number, P: number, Q: number, M: number, N: number): {x: number, y: number} {
        // Calculate the ratio of x and y in the original coordinates
        const xRatio = x / P;
        const yRatio = y / Q;

        // Apply the calculated ratio to the target resolution
        const mappedX = xRatio * M;
        const mappedY = yRatio * N;

        return { x: mappedX, y: mappedY};
    }

    const handleCanvasClick = (event) => {
        const rect = canvasRef.current.getBoundingClientRect();
        console.log(rect)
        const x = Math.round((event.clientX - rect.left) * 1000) / 1000;
        const y = Math.round((event.clientY - rect.top) * 1000) / 1000;

        console.log("x",x)
        console.log("y",y)

        // Example usage:
        const P = 791.375;
        const Q = 395.6875;
        const M = 303;
        const N = 367;

        const mappedCoordinates = mapCoordinates(x, y, P, Q, M, N);

        // Default values for orientation
        const quatX = 0;
        const quatY = 0;
        const quatZ = 0;
        const quatW = 1;

        sendWebSocketRequest('navigateToPoint', {
            x,
            y,
            quatX,
            quatY,
            quatZ,
            quatW
        });
    }

    useEffect(() => {
        const canvas = canvasRef.current;
        canvas.addEventListener('click', handleCanvasClick);

        return () => {
            canvas.removeEventListener('click', handleCanvasClick);
        }
    }, []);

    // Map drawing
    const canvasRef = useRef(null);
    useEffect(() => {
        const canvas: HTMLCanvasElement = canvasRef.current!;
        const context = canvas.getContext("2d")!;

        // Make it higher resolution
//         const pixelRatio = 1;
//         canvas.width *= pixelRatio;
//         canvas.height *= pixelRatio;
//         console.log(canvas.width)
//         console.log(canvas.height)

        const originalWidth = 300;
        const originalHeight = 150;

        // Make it higher resolution
        const pixelRatio = 2;
        canvas.width = originalWidth * pixelRatio;
        canvas.height = originalHeight * pixelRatio;

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
                const xPad = (canvas.width / 2) - (image.width / 2) * scale;
                const yPad = (canvas.height / 2) - (image.height / 2) * scale;

                context.drawImage(image, xPad, yPad, image.width * scale, image.height * scale);
                console.log("image.width",image.width)
                console.log("image.height",image.height)
                console.log("xPad",xPad)
                console.log("yPad",yPad)
                // Draw the robot position if it exists
                if (poseData) {
                    const robotX = xPad + poseData.robotPixelPosition.x * scale;
                    const robotY = yPad + poseData.robotPixelPosition.y * scale;
                    console.log("poseData.robotPixelPosition.x",poseData.robotPixelPosition.x)
                    console.log("poseData.robotPixelPosition.y",poseData.robotPixelPosition.y)
                    console.log("robotX",robotX)
                    console.log("robotY",robotY)
                    context.fillStyle = 'red';
                    context.beginPath();
                    // Adjust the size of the red dot to be appropriately visible
                    const radius = 7;
                    context.arc(robotX, robotY, radius, 0, 2 * Math.PI);
                    context.fill();
                }

                // Draw green dots for key locations
                Object.values(keyLocations).forEach(location => {
                    const locX = xPad + location.x * scale;
                    const locY = yPad + location.y * scale;
                    context.fillStyle = 'green';
                    context.beginPath();
                    context.arc(locX, locY, 5, 0, 2 * Math.PI);
                    context.fill();
                });

            }
        }
    },
        [mapImageUrl, poseData, keyLocations]
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
                    onClick={handleGoToLocation}>
                    Go To Location
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

            <canvas ref={canvasRef} className='Map w-11/12' onClick={handleCanvasClick} />

            {responseMsg && <div className="alert">{responseMsg}</div>}

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
