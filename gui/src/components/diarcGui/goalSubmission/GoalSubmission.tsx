// IMPORTS //
import React, { useEffect, useState } from "react";

// NPM packages
import { SendMessage, ReadyState } from "react-use-websocket";
import "allotment/dist/style.css"
import { Allotment } from "allotment";
import { flattenTree } from "react-accessible-treeview";
import { Tab, TabList, TabPanel, Tabs } from "react-tabs";

// Internal imports
import ActionForm from "./ActionForm";
import GoalForm from "./GoalForm";
import ActionFormContext from "./ActionFormContext";
import ConnectionIndicator from "../util/ConnectionIndicator";
import ActionDatabase from "./ActionDatabase";

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

/**
 * @author Lucien Bao
 * @version 1.0
 * GoalSubmission. Provides a GUI to browse available actions and submit actions and goals.
 * 
 * Action filtering logic adapted from the example at
 * https://dgreene1.github.io/react-accessible-treeview/docs/examples-Filtering.
 */
const GoalSubmission: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState
}) => {
    // STATE //
    // Action browser + action form state
    const [baseActionList, setBaseActionList] = useState<any[]>(flattenTree({
        name: "",
        children: []
    }));
    const [actionList, setActionList] = useState<any[]>(baseActionList.slice());
    const [actionFormContext, setActionFormContext] = useState<string[]>([]);
    const [selectedIds, setSelectedIds] = useState<Set<number>>(new Set());

    // Method to only show nodes containing the filter string
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
    // Submit buttons
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

    // INPUT //
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

    // DOM //
    return (
        <ActionFormContext.Provider value={actionFormContext}>
            <div className="min-h-0 grow w-full flex flex-col border shadow-md
                            border-1 border-[#d1dbe3] items-center p-5 gap-5
                            rounded-md justify-items-stretch">

                {/* Fully tabbed view, for mobile */}
                <Tabs
                    className="displayFlexImportant flex-col min-h-0 grow w-full"
                    id="mediumHide">
                    <TabList className="select-none">
                        <Tab>Action Database</Tab>
                        <Tab>Submit Action</Tab>
                        <Tab>Submit Goal</Tab>
                    </TabList>

                    <TabPanel className="hidden flex-col min-h-0 grow">
                        <Allotment
                            className="h-full overflow-scroll border border-1 md:block
                               border-[#d1dbe3] shadow-md rounded-md hidden"
                            minSize={150}
                            snap>
                            {/*TODO: left off here adding the ManagePresets
                            into a vertical allotment*/}
                            <ActionDatabase
                                actionList={actionList}
                                setActionFormContext={setActionFormContext}
                                setSelectedIds={setSelectedIds}
                                filterNodes={filterNodes}
                                handleExport={handleExport}
                                exportStatus={exportStatus}
                            />
                        </Allotment>
                    </TabPanel>
                    <TabPanel className="hidden flex-col min-h-0 grow">
                        <div className="w-full h-full overflow-y-auto">
                            <ActionForm
                                sendMessage={sendMessage}
                                path={path}
                                submissionStatus={submissionStatus}
                                setSubmissionStatus={setSubmissionStatus}
                            />
                        </div>
                    </TabPanel>
                    <TabPanel className="hidden flex-col min-h-0 grow">
                        <div className="w-full h-full overflow-y-auto">
                            <GoalForm
                                sendMessage={sendMessage}
                                path={path}
                                submissionStatus={submissionStatus}
                                setSubmissionStatus={setSubmissionStatus}
                            />
                        </div>
                    </TabPanel>
                </Tabs>

                {/* Split pane, for wide screens */}
                {/* Horizontal split pane */}
                <Allotment
                    className="h-full overflow-scroll border border-1 md:block
                               border-[#d1dbe3] shadow-md rounded-md hidden"
                    minSize={150}
                    snap>

                    {/* Left pane */}
                    <Allotment.Pane
                        preferredSize={"50%"}
                        minSize={350}
                        className="flex flex-col">
                        <ActionDatabase
                            actionList={actionList}
                            setActionFormContext={setActionFormContext}
                            setSelectedIds={setSelectedIds}
                            filterNodes={filterNodes}
                            handleExport={handleExport}
                            exportStatus={exportStatus}
                        />
                    </Allotment.Pane>

                    {/* Right pane */}
                    <Allotment.Pane
                        className="overflow-y-auto"
                        minSize={300}>

                        <div className="w-full h-full overflow-y-auto p-5">
                            <Tabs className="h-full flex flex-col">
                                <TabList className="select-none">
                                    <Tab>Submit Action</Tab>
                                    <Tab>Submit Goal</Tab>
                                </TabList>

                                <TabPanel className="hidden flex-col min-h-0 grow overflow-y-auto">
                                    <ActionForm
                                        sendMessage={sendMessage}
                                        path={path}
                                        submissionStatus={submissionStatus}
                                        setSubmissionStatus={setSubmissionStatus}
                                    />
                                </TabPanel>

                                <TabPanel className="hidden flex-col min-h-0 grow">
                                    <GoalForm
                                        sendMessage={sendMessage}
                                        path={path}
                                        submissionStatus={submissionStatus}
                                        setSubmissionStatus={setSubmissionStatus}
                                    />
                                </TabPanel>
                            </Tabs>
                        </div>
                    </Allotment.Pane>
                </Allotment>

                <ConnectionIndicator readyState={readyState} />
            </div>
        </ActionFormContext.Provider>
    );
};

export default GoalSubmission;