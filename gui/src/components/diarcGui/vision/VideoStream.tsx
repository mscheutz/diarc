/**
 * @author Lucien Bao
 * @version 0.9
 * @date 21 June 2024
 * VideoStream. Defines a video player component for the vision subproject.
 */

import React, { useState, useEffect, useRef } from "react";

import { Button } from "../../Button";
import ConnectionIndicator from "../util/ConnectionIndicator";

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState,
    trackerDisplay: boolean
};

const VideoStream: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState, trackerDisplay
}) => {

    const [videoEnabled, setVideoEnabled] = useState(false);
    const [depthEnabled, setDepthEnabled] = useState(false);
    const videoCanvasRef = useRef(null);
    const depthCanvasRef = useRef(null);

    // Handle WebSocket messages
    useEffect(() => {
        if (!lastMessage || !lastMessage.data) return;
        try {
            const message = JSON.parse(lastMessage.data);
            if (!message.path || message.path !== path) return;
            const imageData = message.imageData;
            const depthData = message.depthData;
            const detections = message.detectionData || []; // Now expects an array, even if empty

            // Image stream handling
            if (videoCanvasRef.current && videoEnabled) {
                const canvas = videoCanvasRef.current;
                const ctx = canvas.getContext('2d');
                const image = new Image();
                image.onload = () => {
                    canvas.width = image.width;
                    canvas.height = image.height;
                    ctx.clearRect(0, 0, canvas.width, canvas.height);
                    ctx.drawImage(image, 0, 0, canvas.width, canvas.height);

                    if (trackerDisplay) {
                        detections.forEach(detection => {
                            if (detection && detection.boundingBox) {
                                ctx.beginPath();
                                ctx.rect(
                                    detection.boundingBox.x,
                                    detection.boundingBox.y,
                                    detection.boundingBox.width,
                                    detection.boundingBox.height
                                );
                                ctx.strokeStyle = 'blue';
                                ctx.lineWidth = 5;
                                ctx.stroke();
                            }
                        });
                    }
                };
                image.src = `data:image/jpeg;base64,${imageData}`;
            } else if (depthCanvasRef.current && depthEnabled) {
                const canvas = depthCanvasRef.current;
                const ctx = canvas.getContext('2d');
                const image = new Image();
                image.onload = () => {
                    canvas.width = image.width;
                    canvas.height = image.height;
                    ctx.clearRect(0, 0, canvas.width, canvas.height);
                    ctx.drawImage(image, 0, 0, canvas.width, canvas.height);
                };
                image.src = `data:image/png;base64,${depthData}`;
            } else {
                if (videoCanvasRef.current) {
                    const canvas = videoCanvasRef.current;
                    const ctx = canvas.getContext('2d');
                    ctx.clearRect(0, 0, canvas.width, canvas.height); // Clear canvas if both streams are disabled
                }
                if (depthCanvasRef.current) {
                    const canvas = depthCanvasRef.current;
                    const ctx = canvas.getContext('2d');
                    ctx.clearRect(0, 0, canvas.width, canvas.height); // Clear canvas if both streams are disabled
                }
            }
        } catch (error) {
            console.error("Error parsing JSON or setting video source:", error);
        }
    }, [path, lastMessage, videoEnabled, depthEnabled, trackerDisplay]);


    const toggleVideoStream = () => {
        if (videoEnabled) {
            sendMessage(JSON.stringify({ path: path, action: 'stopStream' }));
            setVideoEnabled(false);
        } else {
            sendMessage(JSON.stringify({ path: path, action: 'stopDepthStream' }));
            setDepthEnabled(false);
            sendMessage(JSON.stringify({ path: path, action: 'startStream' }));
            setVideoEnabled(true);
        }
    };

    const toggleDepthStream = () => {
        if (depthEnabled) {
            sendMessage(JSON.stringify({ path: path, action: 'stopDepthStream' }));
            setDepthEnabled(false);
        } else {
            sendMessage(JSON.stringify({ path: path, action: 'stopStream' }));
            setVideoEnabled(false);
            sendMessage(JSON.stringify({ path: path, action: 'startDepthStream' }));
            setDepthEnabled(true);
        }
    };

    const handleScreenshot = () => {
        sendMessage(JSON.stringify({ path: path, action: 'takeSnapshot' }));
    };

    return (
        <div className="min-h-0 grow w-full flex flex-col outline shadow-md
                      outline-1 outline-[#d1dbe3] items-center p-5 gap-5
                      rounded-md justify-items-stretch">
            {/* Video stream/placeholder */}
            {videoEnabled ? (
                <canvas ref={videoCanvasRef} alt="Video Stream" />
            ) : depthEnabled ? (
                <canvas ref={depthCanvasRef} alt="Depth Stream" />
            ) : (
                <div className="min-h-0 min-w-0 grow h-full w-full
                                outline outline-1 outline-[#d1dbe3]
                                rounded-md shadow-md bg-gray-300
                                items-center justify-center flex flex-col">
                    <p className="text-lg">Capture has been disabled.</p>
                </div>
            )}
            {/* Menu */}
            <div className="shadow-md outline outline-1 outline-[#d1dbe3]
                            p-3 rounded-md justify-center gap-3 flex flex-row
                            w-full items-center">

                <Button
                    type="button"
                    title="Enable/disable stream"
                    onClick={toggleVideoStream}
                >
                    {videoEnabled ? 'Disable Capture' : 'Enable Capture'}
                </Button>

                <Button
                    type="button"
                    title="Take screenshot"
                    onClick={handleScreenshot}
                >
                    Take screenshot
                </Button>

                <Button
                    type="button"
                    title="Toggle depth stream"
                    onClick={toggleDepthStream}
                >
                    {depthEnabled ? 'Disable Depth' : 'Enable Depth'}
                </Button>
            </div>

            <div className="hidden">
                <ConnectionIndicator readyState={readyState} />
            </div>
        </div>
    );
};

export default VideoStream;