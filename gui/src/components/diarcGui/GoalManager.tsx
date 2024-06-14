/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * GoalManager. Provides a GUI to view and edit Action Script Language (.asl)
 * files and to submit actions and goals.
 * 
 * Action filtering logic adapted from the example at
 * https://dgreene1.github.io/react-accessible-treeview/docs/examples-Filtering.
 */

import React, { useEffect, useState } from "react";

import useWebSocket from "react-use-websocket";

import "allotment/dist/style.css"
import { Allotment } from "allotment";

import { flattenTree } from "react-accessible-treeview";

import FileBrowser from "./FileBrowser";
import ActionGoalForm from "./ActionGoalForm";
import ActionFormContext from "./ActionFormContext";
import ConnectionIndicator from "./ConnectionIndicator";
import ActionDatabase from "./ActionDatabase";
import { Tab, TabList, TabPanel, Tabs } from "react-tabs";

const GoalManager = () => {
    // Action list state
    const [baseActionList, setBaseActionList] = useState<any[]>(flattenTree({
        name: "",
        children: []
    }));
    const [actionList, setActionList] = useState<any[]>(baseActionList.slice());
    const [actionFormContext, setActionFormContext] = useState<string[]>([]);
    const [selectedIds, setSelectedIds] = useState<Set<number>>(new Set());

    const [fileTree, setFileTree] = useState<object>({
        id: "0",
        name: "asl",
        children: [
            { id: "1", name: "subfolder", children: [] }
        ]
    });

    const filterNodes = (value: string) => {
        if (value && value !== "") {
            const filtered: any[] = [];
            baseActionList.forEach((item) => {
                if (item.id === 0) {
                    return;
                }
                if (item.name.toUpperCase().includes(value.toUpperCase())) {
                    filtered.push(item);
                }
            });
            filtered.unshift(
                Object.assign({
                    ...baseActionList[0],
                    children: baseActionList[0].children.filter((id: any) =>
                        filtered.find((fitem) => fitem.id === id)
                    ),
                })
            );
            setActionList(filtered);
        } else {
            setActionList(baseActionList.slice());
        }
    };

    // Export button
    const [exportStatus, setExportStatus] = useState<string>("");

    const handleExport = () => {
        setExportStatus("wait");
        let array: number[] = [];
        // @ts-ignore
        for (const [value] of selectedIds.entries()) {
            array.push(value);
        }
        sendMessage(JSON.stringify({ "type": "export", "selected": array }));
    };

    // Configure websocket
    const url: URL = new URL(document.location.toString());
    url.port = "8080";
    url.protocol = "ws";

    const wsBaseUrl = url.toString();
    const { sendMessage, lastMessage, readyState } =
        useWebSocket(`${wsBaseUrl}goalManager`);

    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);

        if (data.actions) {
            setBaseActionList(flattenTree({
                name: "", children: data.actions
            }));
            setActionList(flattenTree({
                name: "", children: data.actions
            }));
        }
        if (data.files) {
            setFileTree({ name: "", children: [data.files] });
        }
        if (data.export) {
            setExportStatus("successful")
        }
    },
        [lastMessage]);


    // Render
    return (
        <ActionFormContext.Provider value={actionFormContext}>
            <div className="h-[40rem] max-h-[40rem] w-full flex flex-col outline shadow-md
                      outline-1 outline-[#d1dbe3] items-center p-5 gap-5
                      rounded-md justify-items-stretch">
                {/* Fully tabbed view, for mobile */}
                <Tabs className="max-h-full shrink md:hidden flex flex-col">
                    <TabList>
                        <Tab>Action Database</Tab>
                        <Tab>File Browser</Tab>
                        <Tab>Submit Action</Tab>
                        <Tab>Submit Goal</Tab>
                    </TabList>

                    <TabPanel className="flex flex-col flex-1 w-[90vw] min-h-0 shrink">
                        <ActionDatabase
                            actionList={actionList}
                            setActionFormContext={setActionFormContext}
                            setSelectedIds={setSelectedIds}
                            filterNodes={filterNodes}
                            handleExport={handleExport}
                            exportStatus={exportStatus}
                        />
                    </TabPanel>
                    <TabPanel>
                        <FileBrowser
                            fileTree={fileTree}
                        />
                    </TabPanel>
                    <TabPanel>
                        <div className="w-full h-full overflow-y-auto">
                            <ActionGoalForm sendMessage={sendMessage} />
                        </div>
                    </TabPanel>
                </Tabs>

                {/* Split pane, for wide screens */}
                {/* Horizontal split pane */}
                <Allotment
                    className="h-full overflow-scroll outline outline-1
                     outline-[#d1dbe3] shadow-md rounded-md hidden md:block"
                    minSize={150}
                    snap
                >
                    {/* Left pane */}
                    <Allotment.Pane
                        preferredSize={"50%"}
                        minSize={350}
                    >
                        {/* Vertical split pane */}
                        <Allotment vertical snap>
                            {/* Top pane */}
                            <Allotment.Pane
                                preferredSize={"60%"}
                                minSize={150}
                                className="flex flex-col"
                            >
                                <ActionDatabase
                                    actionList={actionList}
                                    setActionFormContext={setActionFormContext}
                                    setSelectedIds={setSelectedIds}
                                    filterNodes={filterNodes}
                                    handleExport={handleExport}
                                    exportStatus={exportStatus}
                                />
                            </Allotment.Pane>

                            {/* Bottom pane */}
                            <Allotment.Pane minSize={150}>
                                <FileBrowser
                                    fileTree={fileTree}
                                />
                            </Allotment.Pane>
                        </Allotment>
                    </Allotment.Pane>

                    {/* Right pane */}
                    <Allotment.Pane
                        className="overflow-y-auto"
                        minSize={300}
                    >
                        <div className="w-full h-full overflow-y-auto">
                            <ActionGoalForm sendMessage={sendMessage} />
                        </div>
                    </Allotment.Pane>
                </Allotment>

                <ConnectionIndicator readyState={readyState} />
            </div>
        </ActionFormContext.Provider>
    );
};

export default GoalManager;