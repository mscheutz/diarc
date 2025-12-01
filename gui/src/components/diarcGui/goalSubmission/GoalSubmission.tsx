import React, { useEffect, useState } from "react";

import { SendMessage, ReadyState } from "react-use-websocket";
import "allotment/dist/style.css"
import { Allotment } from "allotment";
import { flattenTree } from "react-accessible-treeview";
import { Tab, TabList, TabPanel, Tabs } from "react-tabs";

import ActionForm from "./ActionForm";
import GoalForm from "./GoalForm";
import ActionFormContext from "./ActionFormContext";
import ConnectionIndicator from "../util/ConnectionIndicator";
import ActionDatabase from "./ActionDatabase";
import ManagePresets from "./ManagePresets";
import SubmissionStatusIndicator from "./SubmissionStatusIndicator";

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

const GoalSubmission: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState
}) => {
    
    const [baseActionList, setBaseActionList] = useState<any[]>(flattenTree({
        name: "",
        children: []
    }));
    const [actionList, setActionList] = useState<any[]>(baseActionList.slice());
    const [actionFormContext, setActionFormContext] = useState<string[]>([]);
    const [selectedIds, setSelectedIds] = useState<Set<number>>(new Set());

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

    const [exportStatus, setExportStatus] = useState<string>("");
    const [lastGoalSubmitted, setLastGoalSubmitted] = useState<string>("");
    const [submissionStatus, setSubmissionStatus] = useState<string>("");

    const handleExport = () => {
        setExportStatus("wait");
        let array: number[] = [];
        // @ts-ignore
        for (const [value] of selectedIds.entries()) {
            array.push(value);
        }
        sendMessage(JSON.stringify({
            "type": "export",
            "selected": array,
            "path": path
        }));
    };

    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);
        if (!data.path || data.path !== path) return;

        if (data.actions) {
            setBaseActionList(flattenTree({
                name: "", children: data.actions
            }));
            setActionList(flattenTree({
                name: "", children: data.actions
            }));
        }
        if (data.export) {
            setExportStatus("successful");
        }
        if (data.submit) {
            setSubmissionStatus("successful");
        }
    },
        [lastMessage, path]);

    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            type: "startup"
        }));
    }, [path, sendMessage]);

    return (
        <ActionFormContext.Provider value={actionFormContext}>
            <div className="flex flex-col h-full min-h-0 gap-4">
                <Tabs
                    className="displayFlexImportant flex-col flex-1 min-h-0 md:hidden"
                    id="mediumHide"
                >
                    <TabList className="select-none">
                        <Tab>Database</Tab>
                        <Tab>Action</Tab>
                        <Tab>Goal</Tab>
                    </TabList>

                    <TabPanel className="hidden flex-col flex-1 min-h-0 gap-4 pt-4">
                        <div className="flex-1 min-h-0 border border-border rounded-lg overflow-hidden bg-card">
                            <ActionDatabase
                                actionList={actionList}
                                setActionFormContext={setActionFormContext}
                                setSelectedIds={setSelectedIds}
                                filterNodes={filterNodes}
                                handleExport={handleExport}
                                exportStatus={exportStatus}
                            />
                        </div>
                        <div className="h-48 min-h-0 border border-border rounded-lg overflow-hidden bg-card">
                            <div className="px-3 py-2 border-b border-border bg-muted/30">
                                <span className="text-xs font-medium text-muted-foreground uppercase tracking-wide">
                                    Saved Presets
                                </span>
                            </div>
                            <ManagePresets
                                sendMessage={sendMessage}
                                path={path}
                                setLastGoalSubmitted={setLastGoalSubmitted}
                                setSubmissionStatus={setSubmissionStatus}
                            />
                        </div>
                    </TabPanel>

                    <TabPanel className="hidden flex-col flex-1 min-h-0 pt-4 overflow-auto">
                        <ActionForm
                            sendMessage={sendMessage}
                            path={path}
                            setLastGoalSubmitted={setLastGoalSubmitted}
                            setSubmissionStatus={setSubmissionStatus}
                        />
                    </TabPanel>

                    <TabPanel className="hidden flex-col flex-1 min-h-0 pt-4 overflow-auto">
                        <GoalForm
                            sendMessage={sendMessage}
                            path={path}
                            setLastGoalSubmitted={setLastGoalSubmitted}
                            setSubmissionStatus={setSubmissionStatus}
                        />
                    </TabPanel>
                </Tabs>

                <Allotment
                    className="hidden md:flex flex-1 min-h-0 rounded-lg border border-border overflow-hidden"
                    minSize={200}
                    snap
                >
                    <Allotment.Pane preferredSize="50%" minSize={300}>
                        <Allotment vertical minSize={100} snap>
                            <Allotment.Pane preferredSize="65%" minSize={200}>
                                <div className="h-full bg-card">
                                    <ActionDatabase
                                        actionList={actionList}
                                        setActionFormContext={setActionFormContext}
                                        setSelectedIds={setSelectedIds}
                                        filterNodes={filterNodes}
                                        handleExport={handleExport}
                                        exportStatus={exportStatus}
                                    />
                                </div>
                            </Allotment.Pane>
                            <Allotment.Pane preferredSize="35%" minSize={100}>
                                <div className="h-full flex flex-col bg-card border-t border-border">
                                    <div className="px-3 py-2 border-b border-border bg-muted/30">
                                        <span className="text-xs font-medium text-muted-foreground uppercase tracking-wide">
                                            Saved Presets
                                        </span>
                                    </div>
                                    <ManagePresets
                                        sendMessage={sendMessage}
                                        path={path}
                                        setLastGoalSubmitted={setLastGoalSubmitted}
                                        setSubmissionStatus={setSubmissionStatus}
                                    />
                                </div>
                            </Allotment.Pane>
                        </Allotment>
                    </Allotment.Pane>

                    <Allotment.Pane minSize={280}>
                        <div className="h-full bg-card border-l border-border overflow-auto">
                            <Tabs className="h-full flex flex-col">
                                <div className="px-4 pt-4">
                                    <TabList className="select-none">
                                        <Tab>Action</Tab>
                                        <Tab>Goal</Tab>
                                    </TabList>
                                </div>

                                <TabPanel className="hidden flex-col flex-1 min-h-0 p-4 overflow-auto">
                                    <ActionForm
                                        sendMessage={sendMessage}
                                        path={path}
                                        setLastGoalSubmitted={setLastGoalSubmitted}
                                        setSubmissionStatus={setSubmissionStatus}
                                    />
                                </TabPanel>

                                <TabPanel className="hidden flex-col flex-1 min-h-0 p-4 overflow-auto">
                                    <GoalForm
                                        sendMessage={sendMessage}
                                        path={path}
                                        setLastGoalSubmitted={setLastGoalSubmitted}
                                        setSubmissionStatus={setSubmissionStatus}
                                    />
                                </TabPanel>
                            </Tabs>
                        </div>
                    </Allotment.Pane>
                </Allotment>

                <div className="flex items-center gap-3 shrink-0">
                    <ConnectionIndicator readyState={readyState} />
                    <SubmissionStatusIndicator goal={lastGoalSubmitted} submissionStatus={submissionStatus} />
                </div>
            </div>
        </ActionFormContext.Provider>
    );
};

export default GoalSubmission;
