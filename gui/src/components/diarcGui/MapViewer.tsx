import React, { useState, useEffect, useCallback, useRef, useReducer } from 'react';
import { Button } from "../Button";
import { ReadyState, SendMessage } from 'react-use-websocket';
import ConnectionIndicator from './util/ConnectionIndicator';
import "./util/StyleOverrides.css";

type Position = { x: number, y: number, z: number };
type Orientation = { x: number, y: number, z: number, w: number };
type RobotPixelPosition = { x: number, y: number };
type Pose = { position: Position, orientation: Orientation, robotPixelPosition: RobotPixelPosition };
type Location = { x: number, y: number, description?: string };
type Locations = { [key: string]: Location };

export type { Position, Orientation, RobotPixelPosition, Pose, Location, Locations };

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
            <strong>{parseInt(key.substring(key.lastIndexOf('_') + 1), 10)}: </strong>
            {Locations[key].description ? Locations[key].description : ''}
        </p>
    ));
};

function computeClickCoordinates(rect: any, xPad: number, yPad: number, canvas: any, image: any, scale: number) {
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

function mapCoordinates(x: number, y: number, rect: any, xPad: number, yPad: number, canvas: any, image: any, scale: number) {
    const coords = computeClickCoordinates(rect, xPad, yPad, canvas, image, scale);

    const xPercent = (x - coords.clickX_leftmost) / (coords.clickX_rightmost - coords.clickX_leftmost);
    const yPercent = (y - coords.clickY_top) / (coords.clickY_bottom - coords.clickY_top);

    const imgX = xPercent * image.width;
    const imgY = yPercent * image.height;

    return {
        imgX,
        imgY
    };
}

const drawDot = (context: any, x: number, y: number, radius: number, color: string) => {
    context.fillStyle = color;
    context.beginPath();
    context.arc(x, y, radius, 0, 2 * Math.PI);
    context.fill();
};

const drawText = (context: any, text: string, x: number, y: number, scale: number, color: string) => {
    context.fillStyle = color;
    context.font = `${15 * scale}px Arial`;
    context.textAlign = 'center';
    context.textBaseline = 'middle';
    context.fillText(text, x, y);
};

const extractLocationNumber = (key: string) => key.split(':')[0].split('_')[1];

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

const useState2 = (initVal: any) => {
  const setter = useRef((___: any, next: any) => ({ val: next })).current
  const [wrapper, dispatch] = useReducer(setter, { val: initVal })
  const state = wrapper.val
  const setState = dispatch
  return [state, setState]
}

const MapViewer: React.FC<Props> = ({ path, lastMessage, sendMessage, readyState }) => {
    const [poseData, setPoseData] = useState<Pose | null>(null);
    const [responseMsg, setResponseMsg] = useState<string>("");
    const [isChanged, setIsChanged] = useState(false);
    const [keyLocations, setKeyLocations] = useState<Locations>({});
    const [pastLocations, setPastLocations] = useState<Locations>({});
    const [mapImageUrl, setMapImageUrl] = useState<string>("");
    const [refresh, setRefresh] = useState<number>(0);
    const [viewportTransform, setViewportTransform] =  useState<any[] | null>(null);
    const [init, setInit] = useState(false);
    const [mouseDownListener, setMouseDownListener] = useState2(null);
    const [mouseUpListener, setMouseUpListener] = useState2(null);
    const [mouseWheelListener, setMouseWheelListener] = useState2(null);

    const canvasRef = useRef<HTMLCanvasElement>(null);
    const locationsCanvasRef = useRef<HTMLCanvasElement>(null);
    const robotCanvasRef = useRef<HTMLCanvasElement>(null);
    const xPadRef = useRef<number>(0);
    const yPadRef = useRef<number>(0);
    const scaleRef = useRef<number>(1);
    const imageRef = useRef<HTMLImageElement | null>(null);

    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);

        if (!data.path || data.path !== path) return;

        if (data.error) {
            alert(data.error);
            console.error("Error received:", data.error);
            return;
        }

        if (data.mapImageUrl) {
            setMapImageUrl(data.mapImageUrl);
            setRefresh(refresh + 1);
        }
        if (data.position && data.orientation) {
            setPoseData({ position: data.position, orientation: data.orientation, robotPixelPosition: data.robotPixelPosition });
        }
        if (data.keyLocations) {
            setKeyLocations(data.keyLocations);
        }
        if (data.pastLocations) {
            setPastLocations(data.pastLocations);
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

    const sendWebSocketRequest = useCallback((action: string, additionalData: object, path?: string) => {
        const payload: any = { action, ...additionalData };
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
    }, [sendWebSocketRequest, path]);

    useEffect(() => {
        if (init) {
            const canvas = canvasRef.current!;
            const context = canvas.getContext("2d")!;
            const robotCv = robotCanvasRef.current!;
            const robotCtx = robotCv.getContext("2d")!;
            const locationsCv = locationsCanvasRef.current!;
            const locationsCtx = locationsCv.getContext("2d")!;

            const internalViewportT = [0,0,1]
            canvas.width = 1920;
            canvas.height = 1080;
            robotCv.width = 1920;
            robotCv.height = 1080;
            locationsCv.width = 1920;
            locationsCv.height = 1080;

            let previousX = 0, previousY = 0;

            const updatePanning = (e: MouseEvent) => {
              const localX = e.clientX;
              const localY = e.clientY;

              internalViewportT[0] += (localX - previousX)*3;
              internalViewportT[1] += (localY - previousY)*3;

              previousX = localX;
              previousY = localY;

              var temp = internalViewportT.slice()
              setViewportTransform(temp);
            }

             const updateZooming = (e: WheelEvent) => {
              const localX = e.clientX;
              const localY = e.clientY;

              const previousScale = internalViewportT[2];

              const newScale = internalViewportT[2] += e.deltaY * -0.001;

              const newX = localX - (localX - internalViewportT[0]) * (newScale / previousScale);
              const newY = localY - (localY - internalViewportT[1]) * (newScale / previousScale);

              internalViewportT[0] = newX;
              internalViewportT[1] = newY;
              internalViewportT[2] = newScale;
              var temp = internalViewportT.slice()
              setViewportTransform(temp);
            }

            const onMouseMove = (e: any) => {
              updatePanning(e)
            }

            const onMouseDown = (e: any) => {
                previousX = e.clientX;
                previousY = e.clientY;

                robotCv.addEventListener("mousemove", onMouseMove);
            };
            const onMouseUp = (e: any) => {
                robotCv.removeEventListener("mousemove", onMouseMove);
            };
            const onMouseWheel = (e: any) => {
                updateZooming(e)
            }

            if (mouseDownListener !== null) {
                robotCv.removeEventListener("mousedown", mouseDownListener);
                robotCv.removeEventListener("mouseup", mouseUpListener);
                robotCv.removeEventListener("wheel", mouseWheelListener);
            }

            robotCv.addEventListener("mousedown", onMouseDown);
            robotCv.addEventListener("mouseup", onMouseUp);
            robotCv.addEventListener("wheel", onMouseWheel);
            setMouseDownListener(onMouseDown);
            setMouseUpListener(onMouseUp);
            setMouseWheelListener(onMouseWheel);
            setViewportTransform([0,0,1]);
        }
    },
        [init, refresh]
    );

    useEffect(() => {
        setInit(true)
        const canvas = canvasRef.current!;
        const context = canvas.getContext("2d")!;

        if (!mapImageUrl || !init) {
            context.clearRect(0, 0, 1920, 1080);
            context.fillStyle = "#888888";
            context.textBaseline = "middle";
            context.textAlign = "center";
            context.font = "40px Ubuntu";
            context.fillText("Map not loaded", canvas.width / 2, canvas.height / 2);
        } else {
            const image = new Image();
            image.onload = () => {
                const scale = Math.min(canvas.width / image.width, canvas.height / image.height);
                const xPad = (canvas.width / 2) - (image.width / 2) * scale;
                const yPad = (canvas.height / 2) - (image.height / 2) * scale;
                var temp = viewportTransform!.slice()

                context.clearRect(0, 0, 1920, 1080);
                context.setTransform(temp[2], 0, 0, temp[2], temp[0], temp[1]);
                context.drawImage(image, xPad, yPad, image.width * scale, image.height * scale);

                xPadRef.current = xPad;
                yPadRef.current = yPad;
                scaleRef.current = scale;
                imageRef.current = image;
            }
        }
    },
        [mapImageUrl, viewportTransform, init]
    );

    useEffect(() => {
        const robotCv = robotCanvasRef.current;
        if (robotCv != null) {
            const robotCtx = robotCv.getContext("2d")!;
            if (!mapImageUrl || !init) {
                return
            } else {
                if (poseData) {
                    robotCtx.clearRect(0,0,1920,1080)
                    var temp = viewportTransform!.slice()
                    robotCtx.setTransform(temp[2], 0, 0, temp[2], temp[0], temp[1]);
                    const robotX = xPadRef.current + poseData.robotPixelPosition.x * scaleRef.current;
                    const robotY = yPadRef.current + poseData.robotPixelPosition.y * scaleRef.current;
                    drawDot(robotCtx, robotX, robotY, 15 * scaleRef.current, 'red');
                }
            }
        }
    },
        [poseData, viewportTransform, init]
    );

    useEffect(() => {
        const locationsCv = locationsCanvasRef.current;
        if (locationsCv != null) {
            const locationsCtx = locationsCv.getContext("2d")!;

            if (!mapImageUrl || !init) {
                return
            } else {
                locationsCtx.clearRect(0,0,1920,1080)
                var temp = viewportTransform!.slice()
                locationsCtx.setTransform(temp[2], 0, 0, temp[2], temp[0], temp[1]);
                Object.entries(keyLocations).forEach(([key, location]) => {
                    const locX = xPadRef.current + location.x * scaleRef.current;
                    const locY = yPadRef.current + location.y * scaleRef.current;
                    drawDot(locationsCtx, locX, locY, 15 * scaleRef.current, 'green');
                    drawText(locationsCtx, extractLocationNumber(key), locX, locY, scaleRef.current, 'black');
                });

                Object.entries(pastLocations).forEach(([key, location]) => {
                    const locX = xPadRef.current + location.x * scaleRef.current;
                    const locY = yPadRef.current + location.y * scaleRef.current;
                    drawDot(locationsCtx, locX, locY, 5 * scaleRef.current, 'orange');
                    drawText(locationsCtx, extractLocationNumber(key), locX, locY, scaleRef.current, 'white');
                });
            }
        }
    },
        [keyLocations, pastLocations, viewportTransform, init]
    );

    useEffect(() => {
        sendWebSocketRequest('startup', {}, path);
    }, [path, sendMessage]);

    useEffect(() => {
        const pollInterval = setInterval(() => {
            sendWebSocketRequest('fetchRobotPose', {}, path);
        }, 5000);
        return () => clearInterval(pollInterval);
    }, [path, sendWebSocketRequest]);

    return (
        <div className="map-container h-full w-full flex flex-col gap-5 border
                        border-border items-center p-5 rounded-md bg-card">
            <div className="flex flex-row justify-center gap-2 items-center w-full relative">
                <div className="flex flex-row gap-2">
                    <Button onClick={() => sendWebSocketRequest('fetchMapData', {}, path)}>
                        Refresh Map
                    </Button>
                    {/*}<Button onClick={handleGoToLocation}>
                        Go To Location
                    </Button>*/}
                    <Button onClick={() => sendWebSocketRequest('fetchRobotPose', {}, path)}>
                        Update Robot Pose
                    </Button>
                    <Button className="hidden" onClick={() => sendWebSocketRequest('fetchKeyLocations', {}, path)}>
                        Fetch Key Locations
                    </Button>
                    <Button className="hidden" onClick={() => sendWebSocketRequest('fetchPastLocations', {}, path)}>
                        Fetch Past Locations
                    </Button>
                </div>
                {/*
                {responseMsg && (
                    <div className={`font-bold absolute right-0 top-1/2 transform -translate-y-1/2 transition-colors duration-2000 ${isChanged ? 'text-red-500' : 'text-foreground'}`}>
                        {responseMsg}
                    </div>
                )}
                */}
            </div>

            <div className="flex flex-row justify-center gap-5 h-full w-full">
                <div className="Map w-9/12 parent">
                    <div className="w-full h-full z-2">
                    <canvas ref={canvasRef} className="w-full h-full" onClick={handleCanvasClick} />
                    </div>
                    <div className="w-full h-full z-1">
                    <canvas ref={locationsCanvasRef} className="w-full h-full"/>
                    </div>
                    <div className="w-full h-full z-0">
                    <canvas ref={robotCanvasRef} className="w-full h-full"/>
                    </div>
                </div>

                <div className="flex flex-col gap-2 overflow-y-auto max-h-full w-3/12">
                    <div className="pose-data">
                        {/*<div dangerouslySetInnerHTML={{ __html: position2String(poseData) }}></div>
                        <div dangerouslySetInnerHTML={{ __html: orientation2String(poseData) }}></div>*/}
                    </div>

                    <div className="key-locations">
                        <div className="w-full font-bold">Location Key:</div>
                        {Locations2Elements(keyLocations, 'text-foreground')}
                    </div>
                </div>
            </div>

            <ConnectionIndicator readyState={readyState} />
        </div>
    );
};

export default MapViewer;
