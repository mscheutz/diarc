/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * GoalManager. Provides a GUI to view and edit Action Script Language (.asl)
 * files and to submit actions and goals.
 */

import React, { useEffect, useState, useContext } from "react";

import useWebSocket from "react-use-websocket";

import "allotment/dist/style.css"
import { Allotment } from "allotment";

import ActionBrowser from "./ActionBrowser";
import FileBrowser from "./FileBrowser";
import ActionGoalForm from "./ActionGoalForm";
import ActionFormContext from "./ActionFormContext";

const GoalManager = () => {
    const { sendMessage, lastMessage, readyState } =
        useWebSocket("ws://localhost:8080/goalManager");

    const [actionList, setActionList] = useState<string[]>([]);
    const [fileTree, setFileTree] = useState<object>({
        id: "0",
        name: "asl",
        children: [
            { id: "1", name: "subfolder", children: [] }
        ]
    });
    // Not using the code editor anymore (for now)
    // const [fileContents, setFileContents] =
    //     useState<string>("Selected file contents will appear here.");

    const [actionFormContext, setActionFormContext] = useState<string[]>([]);

    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);

        if (data.actions) {
            setActionList(data.actions.sort());
        }
        if (data.files) {
            setFileTree(data.files);
        }
        // if (data.contents) {
        //     setFileContents(data.contents);
        // }
    },
        [lastMessage]);

    // const { ref, width, height } = useResizeObserver();

    return (
        <ActionFormContext.Provider value={actionFormContext}>
            <div className="map-container h-[40rem] w-full flex flex-col gap-3 outline
                        outline-1 outline-[#d1dbe3] items-center">
                {/* Upper split pane */}
                <Allotment className="h-full overflow-scroll">
                    {/* Left split pane */}
                    <Allotment.Pane preferredSize={"50%"}>
                        <Allotment vertical>
                            {/* Action database */}
                            <Allotment.Pane preferredSize={"50%"}>
                                <ActionBrowser
                                    actionList={actionList}
                                    setActionFormContext={setActionFormContext}
                                />
                            </Allotment.Pane>

                            {/* File browser */}
                            <Allotment.Pane>
                                <div className="w-full h-full overflow-y-auto">
                                    <FileBrowser
                                        fileTree={fileTree}
                                        sendMessage={sendMessage}
                                    />
                                </div>
                            </Allotment.Pane>
                        </Allotment>
                    </Allotment.Pane>

                    {/* Right pane: form view */}
                    <Allotment.Pane className="overflow-y-auto">
                        {/* <FileEditor
                        fileContents={fileContents}
                    /> */}
                        <ActionGoalForm />
                    </Allotment.Pane>
                </Allotment>

                {/* Everything else */}
                <div className="outline outline-1 w-full">Bottom stuff</div>
            </div>
        </ActionFormContext.Provider>
    );
};

export default GoalManager;