/**
 * @author Lucien Bao
 * @version 0.9
 * @date 21 June 2024
 * VideoStream. Defines a video player component for the vision subproject.
 */

import React, { useState } from "react";

import { Button } from "../../Button";

import video from "./video.mp4";

const VideoStream = () => {
    const [videoEnabled, setVideoEnabled] = useState<boolean>(true);
    const [fps, setFps] = useState<number>(666);

    const handleClick = () => {
        setVideoEnabled(!videoEnabled);
    };

    return (
        <div className="min-h-0 grow w-full flex flex-col outline shadow-md
                      outline-1 outline-[#d1dbe3] items-center p-5 gap-5
                      rounded-md justify-items-stretch">
            {videoEnabled ?
                <video
                    className="min-h-0 min-w-0 outline outline-1 outline-[#d1dbe3]
                           rounded-md shadow-md"
                    id="received_video"
                    autoPlay
                    loop>
                    <source src={video} type="video/mp4" />
                </video> :
                <div className="min-h-0 min-w-0 grow h-full w-full
                                outline outline-1 outline-[#d1dbe3]
                                rounded-md shadow-md bg-gray-300
                                items-center justify-center flex flex-col">
                    <p className="text-lg">Video stream has been disabled.</p>
                </div>
            }
            <div className="shadow-md outline outline-1 outline-[#d1dbe3]
                            p-3 rounded-md justify-center gap-3 flex flex-row
                            w-full items-center">
                <p>FPS: {fps}</p>

                <Button
                    type="button"
                    title="Enable/disable stream"
                    onClick={handleClick}
                >
                    Enable/disable stream
                </Button>
            </div>
        </div>
    );
};

export default VideoStream;