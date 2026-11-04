import React, { useState, useEffect, useCallback, useRef } from 'react';
import { Button } from "../Button";
import { ReadyState, SendMessage } from 'react-use-websocket';
import ConnectionIndicator from './util/ConnectionIndicator';

type Position = { x: number, y: number, z: number };
type Orientation = { x: number, y: number, z: number, w: number };
type RobotPixelPosition = { x: number, y: number };
type Pose = { position: Position, orientation: Orientation, robotPixelPosition: RobotPixelPosition };
type Location = { x: number, y: number, description?: string };
type Locations = { [key: string]: Location };

export { Position, Orientation, RobotPixelPosition, Pose, Location, Locations };

const position2String = (pose: Pose | null) => {
    if (pose === null)
        return "Position not known";

    return `<strong>Position</strong> <br>X: ${pose.position.x},<br>Y: ${pose.position.y},<br>Z: ${pose.position.z}`;
};

const orientation2String = (pose: Pose | null) => {
    if (pose === null)
        return "Orientation not known";

    return `<strong>Orientation</strong> <br>X: ${pose.orientation.x},<br>Y: ${pose.orientation.y},<br>Z: ${pose.orientation.z},<br>W: ${pose.orientation.w}`;
};

const Locations2Elements = (Locations: Locations, tailwindClass: string): JSX.Element[] => {
    const sortedKeys = Object.keys(Locations).sort((a, b) => {
        const numA = parseInt(a.substring(a.lastIndexOf('_') + 1), 10);
        const numB = parseInt(b.substring(b.lastIndexOf('_') + 1), 10);
        if (numA !== numB) {
            return numA - numB;
        }
        return a.localeCompare(b);
    });

    return sortedKeys.map(key => (
        <p key={key} className={`${tailwindClass}`}>
            <strong>{key}</strong><br />
            {Locations[key].description ? Locations[key].description : ''}
        </p>
    ));
};

function computeClickCoordinates(rect, xPad, yPad, canvas, image, scale) {
    const clickX_leftmost = xPad / canvas.width * rect.width;
    const clickX_rightmost = (xPad + image.width * scale) / canvas.width * rect.width;
    const clickY_top = yPad / canvas.height * rect.height;
    const clickY_bottom = (yPad + image.height * scale) / canvas.height * rect.height;

    return {
        clickX_leftmost,
        clickX_rightmost,
        clickY_top,
        clickY_bottom
    };
}

function mapCoordinates(x, y, rect, xPad, yPad, canvas, image, scale) {
    const coords = computeClickCoordinates(rect, xPad, yPad, canvas, image, scale);

    // Calculate the percentage of x and y within their ranges
    const xPercent = (x - coords.clickX_leftmost) / (coords.clickX_rightmost - coords.clickX_leftmost);
    const yPercent = (y - coords.clickY_top) / (coords.clickY_bottom - coords.clickY_top);

    // Map the percentages to the image's coordinate system
    const imgX = xPercent * image.width;
    const imgY = yPercent * image.height;

    return {
        imgX,
        imgY
    };
}

const drawDot = (context, x, y, radius, color) => {
    context.fillStyle = color;
    context.beginPath();
    context.arc(x, y, radius, 0, 2 * Math.PI);
    context.fill();
};

const drawText = (context, text, x, y, scale, color) => {
    context.fillStyle = color;
    context.font = `${10 * scale}px Arial`;
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillText(text, x, y);
};

const extractLocationNumber = (key) => key.split(':')[0].split('_')[1];

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

const MapViewer: React.FC<Props> = ({ path, lastMessage, sendMessage, readyState }) => {
    // State //
    const [poseData, setPoseData] = useState<Pose | null>(null);
    const [responseMsg, setResponseMsg] = useState<string>("");
    const [isChanged, setIsChanged] = useState(false);
    const [keyLocations, setKeyLocations] = useState<Locations>({});
    const [pastLocations, setPastLocations] = useState<Locations>({});
    const [mapImageUrl, setMapImageUrl] = useState<string>("");

    // State for drawing calculations
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const xPadRef = useRef<number>(0);
    const yPadRef = useRef<number>(0);
    const scaleRef = useRef<number>(1);
    const imageRef = useRef<HTMLImageElement | null>(null);

    // Handle received messages
    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);

        if (!data.path || data.path !== path) return;

        if (data.error) {
            alert(data.error); // Alert the user about the error
            console.error("Error received:", data.error);
            return; // Stop processing further as there was an error
        }

        if (data.mapImageUrl) {
            setMapImageUrl(data.mapImageUrl);
            console.log(data.mapImageUrl)
        }
        if (data.position && data.orientation) {
            setPoseData({ position: data.position, orientation: data.orientation, robotPixelPosition: data.robotPixelPosition });
        }
        if (data.keyLocations) {
            setKeyLocations(data.keyLocations);
            console.log("keyLocations", data.keyLocations)
        }
        if (data.pastLocations) {
            setPastLocations(data.pastLocations);
            console.log("pastLocations", data.pastLocations)
        }
        if (data.message) {
            setResponseMsg(`${data.success ? 'Success: ' : 'Failure: '} ${data.message}`);
            setIsChanged(true);
        }
    }, [lastMessage, path]);

    useEffect(() => {
        if (isChanged) {
            const timeout = setTimeout(() => setIsChanged(false), 2000);
            return () => clearTimeout(timeout);
        }
    }, [isChanged]);

    // Sends various requests to the WebSocket
    const sendWebSocketRequest = useCallback((action: string, additionalData: object, path?: string) => {
        const payload = { action, ...additionalData };
        if (path) {
            payload.path = path;
        }
        sendMessage(JSON.stringify(payload));
    }, [sendMessage]);

    const handleGoToLocation = useCallback(() => {
        const symbol = prompt("Enter the symbol for the location (e.g., location_0:location): ");
        if (symbol && symbol.trim() !== '') {
            sendWebSocketRequest('goToLocation', { locationSymbol: symbol.trim() }, path);
        } else {
            alert("Please enter a valid Symbol for the location in the format of name:type");
        }
    }, [sendWebSocketRequest, path]);

    const handleNavigateToPoint = useCallback(() => {
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
            }, path);
        } else {
            alert("Please enter valid numerical coordinates.");
        }
    }, [sendWebSocketRequest, path]);

    const handleCanvasClick = useCallback((event: React.MouseEvent) => {
        const rect = canvasRef.current?.getBoundingClientRect();
        if (!rect) return;

        const clickX = event.clientX - rect.left;
        const clickY = event.clientY - rect.top;

        // Ensure all references are valid
        if (imageRef.current && xPadRef.current !== undefined && yPadRef.current !== undefined && scaleRef.current !== undefined) {
            const { current: image } = imageRef;
            const { current: xPad } = xPadRef;
            const { current: yPad } = yPadRef;
            const { current: scale } = scaleRef;

            const imageCoordinates = mapCoordinates(clickX, clickY, rect, xPad, yPad, canvasRef.current, image, scale);

            console.log(`Image Coordinates: X=${imageCoordinates.imgX}, Y=${imageCoordinates.imgY}`);

            sendWebSocketRequest('navigateToPoint', {
                x: imageCoordinates.imgX,
                y: imageCoordinates.imgY,
                quatX: 0,   // Assume defaults for orientation
                quatY: 0,
                quatZ: 0,
                quatW: 1
            }, path);

            console.log(`Navigating to Image Coordinates: X=${imageCoordinates.imgX}, Y=${imageCoordinates.imgY}`);
        }
    }, [sendWebSocketRequest, path]);

    // Drawing on the canvas
    useEffect(() => {
        const canvas = canvasRef.current!;
        const context = canvas.getContext("2d")!;

        canvas.width = 1920;
        canvas.height = 1080;

        // Clear existing context
        context.clearRect(0, 0, canvas.width, canvas.height);

        // Background
        context.strokeStyle = "#d1dbe3";
        context.strokeRect(1, 1, canvas.width - 1, canvas.height - 1);

        if (!mapImageUrl) {
            context.fillStyle = "#000000";
            context.textBaseline = "middle";
            context.textAlign = "center";
            context.font = "40px Ubuntu";
            context.fillText("Map not loaded", canvas.width / 2, canvas.height / 2);
        } else {
            const image = new Image();
            image.src = mapImageUrl;
            image.onload = () => {
                // Image scaling logic
                const scale = Math.min(canvas.width / image.width, canvas.height / image.height);
                const xPad = (canvas.width / 2) - (image.width / 2) * scale;
                const yPad = (canvas.height / 2) - (image.height / 2) * scale;

                context.drawImage(image, xPad, yPad, image.width * scale, image.height * scale);

                // Update refs
                xPadRef.current = xPad;
                yPadRef.current = yPad;
                scaleRef.current = scale;
                imageRef.current = image;

                // Draw the robot position if it exists
                if (poseData) {
                    const robotX = xPad + poseData.robotPixelPosition.x * scale;
                    const robotY = yPad + poseData.robotPixelPosition.y * scale;
                    drawDot(context, robotX, robotY, 7 * scale, 'red');
                }

                // Draw Key Locations
                Object.entries(keyLocations).forEach(([key, location]) => {
                    const locX = xPad + location.x * scale;
                    const locY = yPad + location.y * scale;
                    drawDot(context, locX, locY, 5 * scale, 'green');
                    drawText(context, extractLocationNumber(key), locX, locY, scale, 'black');
                });

                // Draw Past Locations
                Object.entries(pastLocations).forEach(([key, location]) => {
                    const locX = xPad + location.x * scale;
                    const locY = yPad + location.y * scale;
                    drawDot(context, locX, locY, 5 * scale, 'orange');
                    drawText(context, extractLocationNumber(key), locX, locY, scale, 'white');
                });
            }
        }
    },
        [mapImageUrl, poseData, keyLocations, pastLocations]
    );

    return (
        <div className="map-container h-full w-full flex flex-col gap-5 outline
                        outline-1 outline-[#d1dbe3] items-center p-5 rounded-md">
            {/* Button menu */}
            <div className="flex flex-row justify-center gap-2 items-center w-full relative">
                <div className="flex flex-row gap-2">
                    <Button onClick={() => sendWebSocketRequest('fetchMapData', {}, path)}>
                        Fetch Map Data
                    </Button>
                    <Button className="hidden" onClick={handleNavigateToPoint}>
                        Navigate To Point
                    </Button>
                    <Button onClick={handleGoToLocation}>
                        Go To Location
                    </Button>
                    <Button onClick={() => sendWebSocketRequest('fetchRobotPose', {}, path)}>
                        Show/Update Robot Pose
                    </Button>
                    <Button className="hidden" onClick={() => sendWebSocketRequest('fetchKeyLocations', {}, path)}>
                        Fetch Key Locations
                    </Button>
                    <Button className="hidden" onClick={() => sendWebSocketRequest('fetchPastLocations', {}, path)}>
                        Fetch Past Locations
                    </Button>
                </div>
                {responseMsg && (
                    <div className={`font-bold absolute right-0 top-1/2 transform -translate-y-1/2 transition-colors duration-2000 ${isChanged ? 'text-red-500' : 'text-black'}`}>
                        {responseMsg}
                    </div>
                )}
            </div>

            <div className="flex flex-row justify-center gap-5 h-full">
                <canvas ref={canvasRef} className="Map w-9/12" onClick={handleCanvasClick} />

                <div className="flex flex-col gap-2 overflow-y-auto max-h-full w-3/12">
                    <div className="pose-data">
                        <div dangerouslySetInnerHTML={{ __html: position2String(poseData) }}></div>
                        <div dangerouslySetInnerHTML={{ __html: orientation2String(poseData) }}></div>
                    </div>

                    <div className="key-locations">
                        {Locations2Elements(keyLocations, 'text-black')}
                    </div>
                </div>
            </div>

            <ConnectionIndicator readyState={readyState} />
        </div>
    );
};

export default MapViewer;
