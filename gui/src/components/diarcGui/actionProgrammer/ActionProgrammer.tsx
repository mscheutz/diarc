// IMPORTS
import React, { useEffect, useState } from "react";

// NPM packages
import { Tab, TabList, TabPanel, Tabs } from "react-tabs";
import ActionBuilder from "./ActionBuilder";
import FileBrowser from "./FileBrowser";
import FileEditor from "./FileEditor";
import { ReadyState, SendMessage } from "react-use-websocket";
import { Allotment } from "allotment";
import ConnectionIndicator from "../util/ConnectionIndicator";

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};
/**
 * ActionProgrammer. A GUI component that allows end users to create and save
 * new action scripts using an editor, as well as build new actions out of
 * existing ones using a form-like interface.
 * @author Lucien Bao
 * @version 1.0
 */
const ActionProgrammer: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState
}) => {
    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);
        if (!data.path || data.path !== path) return;

        if (data.actions) {
            setActionsAvailable(data.actions);
        }
        if (data.files) {
            setFileTree({ name: "", children: [data.files] });
        }
        if (data.filename) {
            setFileName(data.filename);
            setisCustomFile(false);
        }
        if (data.contents) {
            setFileContents(data.contents);
            setisCustomFile(false);
        }
    }, [lastMessage, path]);

    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            method: "" // no action required, just send an update of files
        }))
    }, [path, sendMessage]);

    // File browser
    const [fileTree, setFileTree] = useState<object>({
        id: "0",
        name: "asl",
        children: [{ name: "folder" }]
    });

    // Action builder
    const [actionsAvailable, setActionsAvailable] = useState<string[]>([]);

    // File editor
    const [fileName, setFileName] = useState<string>("");
    const [fileContents, setFileContents] = useState<string>("");
    const [isCustomFile, setisCustomFile] = useState<boolean>(false);

    // DOM //
    return (
        <div className="min-h-0 grow w-full flex flex-col border shadow-md
                        border-1 border-[#d1dbe3] items-center p-5 gap-5
                        rounded-md justify-items-stretch">
            {/* Mobile: tabbed view */}
            <Tabs
                className="displayFlexImportant flex-col min-h-0 grow w-full"
                id="mediumHide"
                forceRenderTabPanel
            >
                <TabList className="select-none">
                    <Tab>File Browser</Tab>
                    <Tab>Action Builder</Tab>
                    <Tab>File Editor</Tab>
                </TabList>

                <TabPanel className="hidden flex-col min-h-0 grow">
                    <FileBrowser
                        fileTree={fileTree}
                        sendMessage={sendMessage}
                        path={path}
                    />
                </TabPanel>

                <TabPanel className="hidden flex-col min-h-0 grow">
                    <ActionBuilder
                        actionsAvailable={actionsAvailable}
                        sendMessage={sendMessage}
                        path={path}
                    />
                </TabPanel>

                <TabPanel className="hidden flex-col min-h-0 grow">
                    <FileEditor
                        fileName={fileName}
                        setFileName={setFileName}
                        fileContents={fileContents}
                        setFileContents={setFileContents}
                        isCustomFile={isCustomFile}
                        setIsCustomFile={setisCustomFile}
                        sendMessage={sendMessage}
                        path={path}
                    />
                </TabPanel>
            </Tabs>

            {/* Large screens: file tree + tabs */}
            <Allotment
                className="h-full overflow-scroll border border-1 border-[#d1dbe3]
                           hidden md:block shadow-md rounded-md"
                snap
            >
                {/* Left pane: file browser */}
                <Allotment.Pane
                    preferredSize="30%"
                    minSize={200}
                    className="flex flex-col"
                >
                    <FileBrowser
                        fileTree={fileTree}
                        sendMessage={sendMessage}
                        path={path}
                    />
                </Allotment.Pane>

                {/* Right panel: tabs for action programmer + file editor */}
                <Allotment.Pane
                    className="overflow-y-auto"
                    minSize={300}
                >
                    <div className="w-full h-full overflow-y-auto p-5">
                        <Tabs
                            className="h-full flex flex-col"
                            forceRenderTabPanel
                        >
                            <TabList className="select-none">
                                <Tab>Action Builder</Tab>
                                <Tab>File Editor</Tab>
                            </TabList>

                            <TabPanel className="hidden flex-col min-h-0 grow">
                                <ActionBuilder
                                    actionsAvailable={actionsAvailable}
                                    sendMessage={sendMessage}
                                    path={path}
                                />
                            </TabPanel>

                            <TabPanel className="hidden flex-col min-h-0 grow">
                                <FileEditor
                                    fileName={fileName}
                                    setFileName={setFileName}
                                    fileContents={fileContents}
                                    setFileContents={setFileContents}
                                    isCustomFile={isCustomFile}
                                    setIsCustomFile={setisCustomFile}
                                    sendMessage={sendMessage}
                                    path={path}
                                />
                            </TabPanel>
                        </Tabs>
                    </div>
                </Allotment.Pane>

            </Allotment>

            <ConnectionIndicator readyState={readyState} />
        </div>
    );
};

export default ActionProgrammer;