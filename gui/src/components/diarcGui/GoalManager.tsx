/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * GoalManager. Provides a GUI to view and edit Action Script Language (.asl)
 * files and to submit actions and goals.
 */

import React, { useEffect, useState } from "react";

import useWebSocket from "react-use-websocket";

import "allotment/dist/style.css"
import { Allotment } from "allotment";

import useResizeObserver from "use-resize-observer";

import ActionBrowser from "./ActionBrowser";
import FileBrowser from "./FileBrowser";

const GoalManager = () => {
    const { sendMessage, lastMessage, readyState } =
        useWebSocket("ws://localhost:8080/goalManager");

    const [actionList, setActionList] = useState<string[]>([]);
    const [fileTree, setFileTree] = useState<object>({
        id: "0",
        name: "asl"
    });

    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);

        if (data.actions) {
            setActionList(data.actions.sort());
        }

        if (data.files) {
            setFileTree(data.files);
            console.debug(data.files);
        }
    },
        [lastMessage]);

    const { ref, width, height } = useResizeObserver();

    return (
        <div className="map-container h-[40rem] w-full flex flex-col gap-3 outline
                        outline-1 outline-[#d1dbe3] items-center">
            {/* Upper split pane */}
            <Allotment className="h-full overflow-scroll">
                {/* Left split pane */}
                <Allotment.Pane preferredSize={"40%"}>
                    <Allotment vertical>
                        {/* Action database */}
                        <Allotment.Pane preferredSize={"60%"}>
                            <div className="w-full h-full overflow-auto">
                                <div className="p-2 text-lg outline outline-1
                                            outline-[#d1dbe3] bg-slate-100">
                                    Action Browser
                                </div>
                                <ActionBrowser actionList={actionList} />
                            </div>
                        </Allotment.Pane>

                        {/* File browser */}
                        <Allotment.Pane>
                            <div className="w-full h-full overflow-y-auto"
                                ref={ref}>
                                <div className="p-2 text-lg outline outline-1
                                            outline-[#d1dbe3] bg-slate-100">
                                    File Browser
                                </div>
                                <FileBrowser fileTree={fileTree} width={width}
                                    height={height} />
                            </div>
                        </Allotment.Pane>
                    </Allotment>
                </Allotment.Pane>

                {/* Right pane: editor view */}
                <Allotment.Pane>
                    <div>hi2</div>
                </Allotment.Pane>
            </Allotment>

            {/* Everything else */}
            <div className="outline outline-1">Bottom stuff</div>
        </div>
    );
};

export default GoalManager;